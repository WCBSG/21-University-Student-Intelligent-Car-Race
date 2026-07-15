"""
config.py — 显式 Config（plain class + 引用）

启动 Config.load() 一次；之后可变。Controller 持有 PidGains 引用，
菜单改字段即生效，无需 update_pid_gains()。

兼容：保留 dict 风格 config[key]，供 Menu 过渡期使用。
"""

import json
import os

CONFIG_FILE = "/flash/config.json"

# OpenART 类别（固定）: 0=沙袋/左, 1=网球/上, 2=熊/右
CLS_LEFT, CLS_UP, CLS_RIGHT = 0, 1, 2


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
# 置信度：场上光照常比实验室暗，略放宽（Cam 仍先滤 >0.70）
    self.min_confidence = 18
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
    self.heading = PidGains(kp=2.0, ki=0.0, kd=0.0, max_out=50.0, deadband=1.0)
    self.tracking_bearing = PidGains(kp=1.5, ki=0.05, kd=0.0, max_out=60.0, deadband=0.02)
    self.tracking = TrackingParams()
    # 磁力计（963）
    self.mag_enabled = False
    self.mag_ox = 0.0
    self.mag_oy = 0.0
    self.mag_oz = 0.0
    # 单车完赛（最终计划.md）
    self.match_target_count = 3
    self.start_layout = 0          # 0=预赛直推; 1=底边中; 2=左下; 3=右下; 4=左边中
    self.push_hdg_ref = 0.0        # 朝场心 H_ref
    # 偏角按 cls 下标: [沙袋/左, 网球/上, 熊/右]
    self.hdg_off = [90.0, 0.0, -90.0]
    self.match_order = [CLS_UP, CLS_LEFT, CLS_RIGHT]  # PICK 优先序
    self.drive_duty = 15.0         # LEAVE/NEXT/HOME/自旋
    self.push_duty = 12.0
    self.drive_timeout_ms = 5000   # LEAVE + NEXT 直行
    self.push_timeout_ms = 3000
    self.next_spin_ms = 1500
    self.home_timeout_ms = 12000
    self.align_tol_deg = 12.0

  def hdg_off_for(self, cls_id):
    """类别 → 相对 H_ref 偏角；未知类返回 0。"""
    i = int(cls_id)
    if 0 <= i < len(self.hdg_off):
      return float(self.hdg_off[i])
    return 0.0

  # —— dict 兼容（Menu）——————————————————————————————

  _KEY_GET = {
    "target_speed": lambda c: c.target_speed,
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
    d["mag_enabled"] = bool(self.mag_enabled)
    d["mag_ox"] = float(self.mag_ox)
    d["mag_oy"] = float(self.mag_oy)
    d["mag_oz"] = float(self.mag_oz)
    d["match_target_count"] = int(self.match_target_count)
    d["start_layout"] = int(self.start_layout)
    d["push_hdg_ref"] = float(self.push_hdg_ref)
    d["hdg_off"] = [float(x) for x in self.hdg_off]
    d["match_order"] = [int(x) for x in self.match_order]
    d["drive_duty"] = float(self.drive_duty)
    d["push_duty"] = float(self.push_duty)
    d["drive_timeout_ms"] = int(self.drive_timeout_ms)
    d["push_timeout_ms"] = int(self.push_timeout_ms)
    d["next_spin_ms"] = int(self.next_spin_ms)
    d["home_timeout_ms"] = int(self.home_timeout_ms)
    d["align_tol_deg"] = float(self.align_tol_deg)
    return d

  def _apply_dict(self, loaded):
    _FLOAT = (
      "mag_ox", "mag_oy", "mag_oz", "push_hdg_ref",
      "drive_duty", "push_duty", "align_tol_deg",
    )
    _INT = (
      "match_target_count", "start_layout",
      "drive_timeout_ms", "push_timeout_ms",
      "next_spin_ms", "home_timeout_ms",
    )
    for k, v in loaded.items():
      if k.startswith("//") or k.startswith("__"):
        continue
      if k == "trk_lost_frames":
        self.tracking.lost_frames = int(v)
      elif k == "cam_timeout_ms":
        self.tracking.cam_timeout_ms = int(v)
      elif k == "mag_enabled":
        self.mag_enabled = bool(v)
      elif k == "hdg_off":
        self.hdg_off = [float(x) for x in v]
      elif k == "match_order":
        self.match_order = [int(x) for x in v]
      elif k in _FLOAT:
        setattr(self, k, float(v))
      elif k in _INT:
        setattr(self, k, int(v))
      elif k in self._KEY_SET:
        self[k] = v
      else:
        # 旧键兼容（一次迁移）
        self._apply_legacy(k, v)

  def _apply_legacy(self, k, v):
    """旧 config.json 字段映射到新结构。"""
    if k == "leave_duty" or k == "spin_duty":
      self.drive_duty = float(v)
    elif k == "leave_timeout_ms" or k == "next_drive_timeout_ms":
      self.drive_timeout_ms = int(v)
    elif k == "hdg_off_left":
      self.hdg_off[CLS_LEFT] = float(v)
    elif k == "hdg_off_up":
      self.hdg_off[CLS_UP] = float(v)
    elif k == "hdg_off_right":
      self.hdg_off[CLS_RIGHT] = float(v)
    elif k == "cls_up":
      # 仅当仍用旧三分字段时重建 match_order 一头
      pass
    elif k == "angle_threshold":
      pass  # 已删除：无控制逻辑使用

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


config = Config()


def load_config():
  """兼容旧 API：原地更新模块级 config。"""
  try:
    with open(CONFIG_FILE, "r") as f:
      loaded = json.load(f)
    config._apply_dict(loaded)
  except (OSError, ValueError):
    try:
      config.save()
    except (OSError, ValueError):
      pass
  return config


def save_config():
  config.save()
