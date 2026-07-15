"""
config.py — 纯比赛固件用 Config（无 Menu / 无 dict 兼容）
启动 Config.load() 从 /flash/config.json 加载；字段均中文 key。
"""

import json
import os

CONFIG_FILE = "/flash/config.json"

# OpenART 类别（固定）: 0=沙袋/左, 1=网球/上, 2=熊/右
CLS_LEFT, CLS_UP, CLS_RIGHT = 0, 1, 2


class PidGains:
  def __init__(self, kp=2.0, ki=0.0, kd=0.0, max_out=50.0, deadband=1.0):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.max_out = max_out
    self.deadband = deadband


class TrackingParams:
  def __init__(self):
    self.approach_speed = 15.0
    self.search_speed = 15.0
    self.target_class = 7
    self.min_confidence = 18
    self.confirm_frames = 4
    self.lost_frames = 4
    self.stop_bottom_pct = 95.0
    self.reverse_angle = 30.0
    self.cam_timeout_ms = 5000


class Config:
  def __init__(self):
    self.target_speed = 50.0
    self.heading = PidGains(kp=2.0, ki=0.0, kd=0.0, max_out=50.0, deadband=1.0)
    self.tracking_bearing = PidGains(kp=1.5, ki=0.05, kd=0.0, max_out=60.0, deadband=0.02)
    self.tracking = TrackingParams()
    # 磁力计
    self.mag_enabled = False
    self.mag_ox = 0.0
    self.mag_oy = 0.0
    self.mag_oz = 0.0
    # 比赛
    self.match_target_count = 3
    self.start_layout = 0           # 0=预赛直推; 1=底边中; 2=左下; 3=右下; 4=左边中
    self.push_hdg_ref = 0.0         # 朝场心 H_ref
    self.hdg_off = [90.0, 0.0, -90.0]  # 按 cls_id: 沙袋/网球/熊
    self.match_order = [CLS_UP, CLS_LEFT, CLS_RIGHT]  # PICK 优先序
    self.drive_duty = 15.0
    self.push_duty = 12.0
    self.drive_timeout_ms = 5000
    self.push_timeout_ms = 3000
    self.push_clear_ms = 200
    self.next_spin_ms = 1500
    self.home_timeout_ms = 12000
    self.align_tol_deg = 12.0

  def hdg_off_for(self, cls_id):
    i = int(cls_id)
    if 0 <= i < len(self.hdg_off):
      return float(self.hdg_off[i])
    return 0.0

  # ——— 序列化（JSON key 全中文）——————————————————————

  def to_dict(self):
    return {
      # 基础
      "默认速度": float(self.target_speed),
      # 航向 PID
      "航向P": float(self.heading.kp),
      "航向I": float(self.heading.ki),
      "航向D": float(self.heading.kd),
      "航向上限": float(self.heading.max_out),
      "航向死区": float(self.heading.deadband),
      # 跟踪 PID
      "跟踪P": float(self.tracking_bearing.kp),
      "跟踪I": float(self.tracking_bearing.ki),
      "跟踪D": float(self.tracking_bearing.kd),
      "跟踪上限": float(self.tracking_bearing.max_out),
      "跟踪死区": float(self.tracking_bearing.deadband),
      # 跟踪参数
      "接近速度": float(self.tracking.approach_speed),
      "搜索速度": float(self.tracking.search_speed),
      "目标类别": int(self.tracking.target_class),
      "最低置信度": int(self.tracking.min_confidence),
      "确认帧数": int(self.tracking.confirm_frames),
      "丢失帧数": int(self.tracking.lost_frames),
      "停止位置": float(self.tracking.stop_bottom_pct),
      "倒车角度": float(self.tracking.reverse_angle),
      "相机超时": int(self.tracking.cam_timeout_ms),
      # 磁力计
      "磁力计开关": bool(self.mag_enabled),
      "磁力计X偏移": float(self.mag_ox),
      "磁力计Y偏移": float(self.mag_oy),
      "磁力计Z偏移": float(self.mag_oz),
      # 比赛
      "目标个数": int(self.match_target_count),
      "发车位置": int(self.start_layout),
      "场心航向": float(self.push_hdg_ref),
      "推箱偏角": [float(x) for x in self.hdg_off],
      "搜索顺序": [int(x) for x in self.match_order],
      "行驶占空比": float(self.drive_duty),
      "推箱占空比": float(self.push_duty),
      "行驶超时": int(self.drive_timeout_ms),
      "推箱超时": int(self.push_timeout_ms),
      "清线时间": int(self.push_clear_ms),
      "掉头超时": int(self.next_spin_ms),
      "回库超时": int(self.home_timeout_ms),
      "航向容差": float(self.align_tol_deg),
    }

  def _apply_dict(self, d):
    for k, v in d.items():
      if k.startswith("//") or k.startswith("__"):
        continue
      try:
        self._set_one(k, v)
      except Exception as e:
        print("[CONFIG] skip '%s': %s" % (k, e))

  def _set_one(self, k, v):
    # 基础
    if k == "默认速度":
      self.target_speed = float(v)
    # 航向 PID
    elif k == "航向P":
      self.heading.kp = float(v)
    elif k == "航向I":
      self.heading.ki = float(v)
    elif k == "航向D":
      self.heading.kd = float(v)
    elif k == "航向上限":
      self.heading.max_out = float(v)
    elif k == "航向死区":
      self.heading.deadband = float(v)
    # 跟踪 PID
    elif k == "跟踪P":
      self.tracking_bearing.kp = float(v)
    elif k == "跟踪I":
      self.tracking_bearing.ki = float(v)
    elif k == "跟踪D":
      self.tracking_bearing.kd = float(v)
    elif k == "跟踪上限":
      self.tracking_bearing.max_out = float(v)
    elif k == "跟踪死区":
      self.tracking_bearing.deadband = float(v)
    # 跟踪参数
    elif k == "接近速度":
      self.tracking.approach_speed = float(v)
    elif k == "搜索速度":
      self.tracking.search_speed = float(v)
    elif k == "目标类别":
      self.tracking.target_class = int(v)
    elif k == "最低置信度":
      self.tracking.min_confidence = int(v)
    elif k == "确认帧数":
      self.tracking.confirm_frames = int(v)
    elif k == "丢失帧数":
      self.tracking.lost_frames = int(v)
    elif k == "停止位置":
      self.tracking.stop_bottom_pct = float(v)
    elif k == "倒车角度":
      self.tracking.reverse_angle = float(v)
    elif k == "相机超时":
      self.tracking.cam_timeout_ms = int(v)
    # 磁力计
    elif k == "磁力计开关":
      self.mag_enabled = bool(v)
    elif k == "磁力计X偏移":
      self.mag_ox = float(v)
    elif k == "磁力计Y偏移":
      self.mag_oy = float(v)
    elif k == "磁力计Z偏移":
      self.mag_oz = float(v)
    # 比赛
    elif k == "目标个数":
      self.match_target_count = int(v)
    elif k == "发车位置":
      self.start_layout = int(v)
    elif k == "场心航向":
      self.push_hdg_ref = float(v)
    elif k == "推箱偏角":
      self.hdg_off = [float(x) for x in v]
    elif k == "搜索顺序":
      self.match_order = [int(x) for x in v]
    elif k == "行驶占空比":
      self.drive_duty = float(v)
    elif k == "推箱占空比":
      self.push_duty = float(v)
    elif k == "行驶超时":
      self.drive_timeout_ms = int(v)
    elif k == "推箱超时":
      self.push_timeout_ms = int(v)
    elif k == "清线时间":
      self.push_clear_ms = int(v)
    elif k == "掉头超时":
      self.next_spin_ms = int(v)
    elif k == "回库超时":
      self.home_timeout_ms = int(v)
    elif k == "航向容差":
      self.align_tol_deg = float(v)

  @classmethod
  def load(cls, path=CONFIG_FILE):
    cfg = cls()
    try:
      with open(path, "r") as f:
        cfg._apply_dict(json.load(f))
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
  try:
    with open(CONFIG_FILE, "r") as f:
      config._apply_dict(json.load(f))
  except (OSError, ValueError):
    try:
      config.save()
    except (OSError, ValueError):
      pass
  return config


def save_config():
  config.save()
