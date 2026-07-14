"""
config.py — 显式 Config（plain class + 引用）

启动 Config.load() 一次；之后可变。Controller 持有 PidGains 引用，
菜单改字段即生效，无需 update_pid_gains()。

兼容：保留 dict 风格 config[key]，供 Menu 过渡期使用。
"""

import json
import os

CONFIG_FILE = "/flash/config.json"


class PidGains:
  """PID 增益对象 — Controller 持引用，不拷贝。"""

  def __init__(self, kp=2.0, ki=0.0, kd=0.0, max_out=50.0, deadband=1.0):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.max_out = max_out
    self.deadband = deadband


class TrackingParams:
  """视觉跟踪非 PID 参数。"""

  def __init__(self):
    self.approach_speed = 15.0
    self.search_speed = 15.0
    self.target_class = 7
    self.min_confidence = 22
    # 按相机帧计数的去抖阈值（约 400ms @ ~10fps 相机）
    self.confirm_frames = 4
    self.lost_frames = 4
    self.stop_bottom_pct = 95.0
    self.reverse_angle = 30.0
    self.cam_timeout_ms = 5000


class Config:
  """
  可变配置单例内容。load() 返回实例；save() 原子写盘。

  Controller 应持有 self.heading / self.tracking_bearing 引用。
  """

  def __init__(self):
    self.target_speed = 50.0
    self.angle_threshold = 10.0
    self.heading = PidGains(kp=2.0, ki=0.0, kd=0.0, max_out=50.0, deadband=1.0)
    self.tracking_bearing = PidGains(kp=1.5, ki=0.05, kd=0.0, max_out=60.0, deadband=0.02)
    self.tracking = TrackingParams()

  # —— dict 兼容（Menu 过渡）——————————————————————————————

  _KEY_GET = {
    "target_speed": lambda c: c.target_speed,
    "angle_threshold": lambda c: c.angle_threshold,
    "heading_kp": lambda c: c.heading.kp,
    "heading_ki": lambda c: c.heading.ki,
    "heading_kd": lambda c: c.heading.kd,
    "heading_max_correction": lambda c: c.heading.max_out,
    "heading_deadband": lambda c: c.heading.deadband,
    "trk_bearing_kp": lambda c: c.tracking_bearing.kp,
    "trk_bearing_ki": lambda c: c.tracking_bearing.ki,
    "trk_bearing_kd": lambda c: c.tracking_bearing.kd,
    "trk_bearing_max": lambda c: c.tracking_bearing.max_out,
    "trk_bearing_db": lambda c: c.tracking_bearing.deadband,
    "trk_approach_speed": lambda c: c.tracking.approach_speed,
    "trk_search_speed": lambda c: c.tracking.search_speed,
    "trk_target_class": lambda c: c.tracking.target_class,
    "trk_min_confidence": lambda c: c.tracking.min_confidence,
    "trk_confirm_frames": lambda c: c.tracking.confirm_frames,
    "trk_stop_bottom_pct": lambda c: c.tracking.stop_bottom_pct,
    "trk_reverse_angle": lambda c: c.tracking.reverse_angle,
  }

  _KEY_SET = {
    "target_speed": lambda c, v: setattr(c, "target_speed", float(v)),
    "angle_threshold": lambda c, v: setattr(c, "angle_threshold", float(v)),
    "heading_kp": lambda c, v: setattr(c.heading, "kp", float(v)),
    "heading_ki": lambda c, v: setattr(c.heading, "ki", float(v)),
    "heading_kd": lambda c, v: setattr(c.heading, "kd", float(v)),
    "heading_max_correction": lambda c, v: setattr(c.heading, "max_out", float(v)),
    "heading_deadband": lambda c, v: setattr(c.heading, "deadband", float(v)),
    "trk_bearing_kp": lambda c, v: setattr(c.tracking_bearing, "kp", float(v)),
    "trk_bearing_ki": lambda c, v: setattr(c.tracking_bearing, "ki", float(v)),
    "trk_bearing_kd": lambda c, v: setattr(c.tracking_bearing, "kd", float(v)),
    "trk_bearing_max": lambda c, v: setattr(c.tracking_bearing, "max_out", float(v)),
    "trk_bearing_db": lambda c, v: setattr(c.tracking_bearing, "deadband", float(v)),
    "trk_approach_speed": lambda c, v: setattr(c.tracking, "approach_speed", float(v)),
    "trk_search_speed": lambda c, v: setattr(c.tracking, "search_speed", float(v)),
    "trk_target_class": lambda c, v: setattr(c.tracking, "target_class", int(v)),
    "trk_min_confidence": lambda c, v: setattr(c.tracking, "min_confidence", int(v)),
    "trk_confirm_frames": lambda c, v: setattr(c.tracking, "confirm_frames", int(v)),
    "trk_stop_bottom_pct": lambda c, v: setattr(c.tracking, "stop_bottom_pct", float(v)),
    "trk_reverse_angle": lambda c, v: setattr(c.tracking, "reverse_angle", float(v)),
  }

  def __getitem__(self, key):
    fn = self._KEY_GET.get(key)
    if fn is None:
      raise KeyError(key)
    return fn(self)

  def __setitem__(self, key, value):
    fn = self._KEY_SET.get(key)
    if fn is None:
      raise KeyError(key)
    fn(self, value)

  def get(self, key, default=None):
    try:
      return self[key]
    except KeyError:
      return default

  def to_dict(self):
    d = {}
    for k in self._KEY_GET:
      d[k] = self[k]
    d["trk_lost_frames"] = self.tracking.lost_frames
    d["cam_timeout_ms"] = self.tracking.cam_timeout_ms
    return d

  def _apply_dict(self, loaded):
    for k, v in loaded.items():
      if k == "trk_lost_frames":
        self.tracking.lost_frames = int(v)
      elif k == "cam_timeout_ms":
        self.tracking.cam_timeout_ms = int(v)
      elif k in self._KEY_SET:
        self[k] = v

  @classmethod
  def load(cls, path=CONFIG_FILE):
    cfg = cls()
    try:
      with open(path, "r") as f:
        loaded = json.load(f)
      cfg._apply_dict(loaded)
    except (OSError, ValueError):
      cfg.save(path)
    return cfg

  def save(self, path=CONFIG_FILE):
    tmp = path + ".tmp"
    try:
      with open(tmp, "w") as f:
        json.dump(self.to_dict(), f)
      os.rename(tmp, path)
    except (OSError, ValueError) as e:
      print("[CONFIG] Save failed:", e)


# 模块级单例 — 启动时 Config.load() 填入；load_config() 兼容旧调用
config = Config()


def load_config():
  """兼容旧 API：原地更新模块级 config（保持对象引用稳定）。"""
  try:
    with open(CONFIG_FILE, "r") as f:
      loaded = json.load(f)
    config._apply_dict(loaded)
  except (OSError, ValueError):
    # 设备上无文件时写默认；PC/无 /flash 时仅保留内存默认
    try:
      config.save()
    except (OSError, ValueError):
      pass
  return config


def save_config():
  """兼容旧 API。"""
  config.save()
