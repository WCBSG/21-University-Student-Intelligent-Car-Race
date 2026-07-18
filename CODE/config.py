"""
config.py — 纯比赛固件 Config
启动 load_config() 从 /flash/config.json 加载；字段均中文 key。
"""

import json
import os
from log import info

CONFIG_FILE = "/flash/config.json"

# OpenART 类别（固定）: 0=沙袋/左, 1=网球/上, 2=熊/右
CLS_LEFT, CLS_UP, CLS_RIGHT = 0, 1, 2


class PidGains:
  def __init__(self, kp=2.0, max_out=50.0, deadband=1.0):
    self.kp = kp
    self.max_out = max_out
    self.deadband = deadband


class TrackingParams:
  def __init__(self):
    self.approach_speed = 15.0
    self.final_approach_speed = 12.0
    self.search_speed = 15.0
    self.target_class = 7
    self.min_confidence = 18
    self.confirm_frames = 4
    self.lost_frames = 4
    self.stop_bottom_pct = 95.0
    self.stage_bottom_pct = 75.0
    self.contact_bottom_pct = 94.0
    self.bearing_actuation_sign = 1.0
    self.cam_timeout_ms = 5000


def _ints(v):
  return [int(x) for x in v]


def _floats(v):
  return [float(x) for x in v]


def _match_mode(v):
  s = str(v).strip().lower()
  return "pre" if s in ("pre", "预赛") else "final"


# (中文key, 属性路径, 转换)  path=None 表示 Config 自身属性；tuple 表示嵌套
_KEY_MAP = (
  ("航向P", ("heading", "kp"), float),
  ("航向上限", ("heading", "max_out"), float),
  ("航向死区", ("heading", "deadband"), float),
  ("跟踪P", ("tracking_bearing", "kp"), float),
  ("跟踪上限", ("tracking_bearing", "max_out"), float),
  ("跟踪死区", ("tracking_bearing", "deadband"), float),
  ("接近速度", ("tracking", "approach_speed"), float),
  ("最终接近速度", ("tracking", "final_approach_speed"), float),
  ("搜索速度", ("tracking", "search_speed"), float),
  ("目标类别", ("tracking", "target_class"), int),
  ("最低置信度", ("tracking", "min_confidence"), int),
  ("确认帧数", ("tracking", "confirm_frames"), int),
  ("丢失帧数", ("tracking", "lost_frames"), int),
  ("停止位置", ("tracking", "stop_bottom_pct"), float),
  ("绕行起始位置", ("tracking", "stage_bottom_pct"), float),
  ("接触位置", ("tracking", "contact_bottom_pct"), float),
  ("视觉执行极性", ("tracking", "bearing_actuation_sign"), float),
  ("相机超时", ("tracking", "cam_timeout_ms"), int),
  ("磁力计开关", "mag_enabled", bool),
  ("磁力计X偏移", "mag_ox", float),
  ("磁力计Y偏移", "mag_oy", float),
  ("磁力计Z偏移", "mag_oz", float),
  ("目标个数", "match_target_count", int),
  ("发车位置", "start_layout", int),
  ("比赛模式", "match_mode", _match_mode),
  ("场心航向", "push_hdg_ref", float),
  ("推箱偏角", "hdg_off", _floats),
  ("搜索顺序", "match_order", _ints),
  ("严格目标", "strict_target", bool),
  ("单车目标类别", "single_target_class", int),
  ("行驶占空比", "drive_duty", float),
  ("推箱占空比", "push_duty", float),
  ("航向执行极性", "yaw_actuation_sign", float),
  ("绕行速度", "orbit_speed", float),
  ("绕行距离P", "orbit_radial_kp", float),
  ("绕行距离上限", "orbit_radial_max", float),
  ("绕行超时", "orbit_timeout_ms", int),
  ("绕行航向容差", "orbit_yaw_tol_deg", float),
  ("绕行居中容差", "orbit_center_tol_pct", float),
  ("绕行确认帧数", "orbit_confirm_frames", int),
  ("绕行丢失帧数", "orbit_lost_frames", int),
  ("绕轴自旋", "orbit_front_spin", float),
  ("绕轴侧移", "orbit_front_slip", float),
  ("绕轴翻转", "orbit_front_flip", bool),
  ("绕物总超时", "approach_cluster_timeout_ms", int),
  ("行驶超时", "drive_timeout_ms", int),
  ("推箱超时", "push_timeout_ms", int),
  ("推箱监护帧数", "push_watch_frames", int),
  ("推箱左容差", "push_cx_left_min", float),
  ("推箱右容差", "push_cx_right_max", float),
  ("推箱纠偏占空比", "push_correct_duty", float),
  ("推箱丢失盲区", "push_lost_blind_ms", int),
  ("后退最少", "backoff_retreat_min_ms", int),
  ("自旋角度", "backoff_spin_deg", float),
  ("后退超时", "recover_backoff_ms", int),
  ("回库超时", "home_timeout_ms", int),
  ("航向容差", "align_tol_deg", float),
  ("调试输出", "debug_output", bool),
  ("黄线确认帧数", "tcs_confirm_n", int),
  ("黄线R下限", "tcs_r_min", float),
  ("黄线G下限", "tcs_g_min", float),
  ("黄线B上限", "tcs_b_max", float),
  ("黄线亮度下限", "tcs_c_min", int),
  ("搜索超时", "pick_timeout_ms", int),
  ("队首换类帧数", "pick_class_frames", int),
  ("回中前进超时", "center_fwd_ms", int),
  ("离线超时", "home_backoff_ms", int),
  ("标定采样数", "imu_calibrate_samples", int),
  ("滤波增益", "imu_beta", float),
  ("陀螺静止阈值", "imu_gyro_still", float),
  ("加计静止阈值", "imu_acc_still", float),
  ("零偏校正速率", "imu_bias_alpha", float),
  ("磁融合速率", "imu_mag_alpha", float),
  ("磁融合死区", "imu_mag_dead", float),
  ("磁融合上限", "imu_mag_pull_max", float),
  ("磁融合静止帧数", "imu_mag_still_need", int),
  ("零偏静止帧数", "imu_still_needed", int),
  ("磁参考LPFa", "imu_mag_lpf_alpha", float),
  ("陀螺刻度", "imu_gyro_scale", float),
  ("转动滤波增益", "imu_spin_beta", float),
  ("转动角速度阈", "imu_spin_dps", float),
)

_KEY_LOOKUP = {k: (path, fn) for k, path, fn in _KEY_MAP}


class Config:
  def __init__(self):
    self.heading = PidGains(kp=2.0, max_out=50.0, deadband=1.0)
    self.tracking_bearing = PidGains(kp=1.5, max_out=60.0, deadband=0.02)
    self.tracking = TrackingParams()
    self.mag_enabled = False
    self.mag_ox = 0.0
    self.mag_oy = 0.0
    self.mag_oz = 0.0
    self.match_target_count = 3
    self.match_mode = "final"
    self.start_layout = 0
    self.push_hdg_ref = 0.0
    self.hdg_off = [0.0, -90.0, 180.0]
    self.match_order = [CLS_UP, CLS_LEFT, CLS_RIGHT]
    self.strict_target = False
    self.single_target_class = CLS_UP
    self.drive_duty = 15.0
    self.push_duty = 12.0
    self.yaw_actuation_sign = -1.0
    self.orbit_speed = 12.0
    self.orbit_radial_kp = 0.6
    self.orbit_radial_max = 10.0
    self.orbit_timeout_ms = 8000
    self.orbit_yaw_tol_deg = 8.0
    self.orbit_center_tol_pct = 8.0
    self.orbit_confirm_frames = 4
    self.orbit_lost_frames = 6
    self.orbit_front_spin = 40.0
    self.orbit_front_slip = 60.0
    self.orbit_front_flip = False
    self.approach_cluster_timeout_ms = 15000
    self.drive_timeout_ms = 5000
    self.push_timeout_ms = 3000
    self.push_watch_frames = 2
    self.push_cx_left_min = 8.0
    self.push_cx_right_max = 78.0
    self.push_correct_duty = 10.0
    self.push_lost_blind_ms = 600
    self.backoff_retreat_min_ms = 450
    self.backoff_spin_deg = 170.0
    self.recover_backoff_ms = 500
    self.home_timeout_ms = 12000
    self.align_tol_deg = 12.0
    self.debug_output = False
    self.tcs_confirm_n = 2
    self.tcs_r_min = 0.28
    self.tcs_g_min = 0.28
    self.tcs_b_max = 0.25
    self.tcs_c_min = 800
    self.pick_timeout_ms = 20000
    self.pick_class_frames = 6
    self.center_fwd_ms = 4000
    self.home_backoff_ms = 1500
    self.imu_calibrate_samples = 100
    self.imu_beta = 0.05
    self.imu_gyro_still = 0.0175
    self.imu_acc_still = 0.05
    self.imu_bias_alpha = 0.002
    self.imu_mag_alpha = 0.002
    self.imu_mag_dead = 2.2
    self.imu_mag_pull_max = 6.7
    self.imu_mag_still_need = 100
    self.imu_still_needed = 100
    self.imu_mag_lpf_alpha = 0.01
    self.imu_gyro_scale = 1.135
    self.imu_spin_beta = 0.01
    self.imu_spin_dps = 40.0

  def hdg_off_for(self, cls_id):
    i = int(cls_id)
    if 0 <= i < len(self.hdg_off):
      return float(self.hdg_off[i])
    return 0.0

  def _get_path(self, path):
    if isinstance(path, tuple):
      return getattr(getattr(self, path[0]), path[1])
    return getattr(self, path)

  def _set_path(self, path, value):
    if isinstance(path, tuple):
      setattr(getattr(self, path[0]), path[1], value)
    else:
      setattr(self, path, value)

  def to_dict(self):
    d = {}
    for k, path, _fn in _KEY_MAP:
      v = self._get_path(path)
      if isinstance(v, list):
        d[k] = list(v)
      else:
        d[k] = v
    return d

  def _apply_dict(self, d):
    for k, v in d.items():
      if k.startswith("//") or k.startswith("__"):
        continue
      entry = _KEY_LOOKUP.get(k)
      if entry is None:
        continue
      path, fn = entry
      try:
        self._set_path(path, fn(v))
      except Exception as e:
        info("CONFIG", "skip '%s': %s" % (k, e))

  def save(self, path=CONFIG_FILE):
    tmp = path + ".tmp"
    try:
      with open(tmp, "w") as f:
        json.dump(self.to_dict(), f)
      os.rename(tmp, path)
    except (OSError, ValueError) as e:
      info("CONFIG", "Save failed: %s" % e)


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
