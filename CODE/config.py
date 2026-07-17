"""
config.py — 纯比赛固件用 Config（无 Menu / 无 dict 兼容）
启动 Config.load() 从 /flash/config.json 加载；字段均中文 key。
"""

import json
import os
from log import info

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
    self.reverse_angle = 30.0
    self.cam_timeout_ms = 5000


class Config:
  def __init__(self):
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
    self.match_mode = "final"        # "pre"=预赛直推; "final"=决赛绕行
    self.start_layout = 0           # 0=底边中; 1=底边中; 2=左下; 3=右下; 4=左边中
    self.push_hdg_ref = 0.0         # 朝场心 H_ref
    self.hdg_off = [90.0, 0.0, -90.0]  # 按 cls_id: 沙袋/网球/熊
    self.match_order = [CLS_UP, CLS_LEFT, CLS_RIGHT]  # PICK 优先序
    self.strict_target = False
    self.single_target_class = CLS_UP
    self.drive_duty = 15.0
    self.push_duty = 12.0
    # 实车标定: 三轮同正值使 yaw 减小，所以绝对航向执行极性为 -1。
    self.yaw_actuation_sign = -1.0
    self.orbit_speed = 12.0
    self.orbit_direction_sign = 1.0
    self.orbit_radial_kp = 0.6
    self.orbit_radial_max = 10.0
    self.orbit_timeout_ms = 8000
    self.orbit_yaw_tol_deg = 8.0
    self.orbit_center_tol_pct = 8.0
    self.orbit_range_tol_pct = 7.0
    self.orbit_confirm_frames = 4
    self.orbit_lost_frames = 6
    self.final_approach_timeout_ms = 5000
    self.approach_cluster_timeout_ms = 15000
    self.drive_timeout_ms = 5000
    self.push_timeout_ms = 3000
    self.push_clear_ms = 200  # 已废弃：黄线直接 BACKOFF，保留键兼容旧 config.json
    # PUSH 视觉防推空：连续 N 帧丢失→找物；过偏→慢纠；左侧容差更宽（副车兜底）
    self.push_watch_frames = 2
    self.push_cx_left_min = 8.0    # cx 下限（偏左更宽容）
    self.push_cx_right_max = 78.0  # cx 上限（偏右更严）
    self.push_correct_duty = 10.0  # 过偏慢纠前向占空比
    self.push_lost_blind_ms = 600  # 推进超过此时长后忽略丢失（贴车头遮挡）
    self.next_spin_ms = 1500       # 原子 BACKOFF 转 180° 超时
    self.recover_backoff_ms = 500  # 原子 BACKOFF 后退离线超时
    self.home_timeout_ms = 12000
    self.align_tol_deg = 12.0
    self.debug_output = False

    # ——— TCS 颜色传感器 ———
    self.tcs_confirm_n = 2          # 黄线确认帧数
    self.tcs_r_min = 0.28           # 黄线 R 下限
    self.tcs_g_min = 0.28           # 黄线 G 下限
    self.tcs_b_max = 0.25           # 黄线 B 上限
    self.tcs_c_min = 800            # 黄线最低亮度

    # ——— 搜索 / 接近超时 ———
    self.pick_timeout_ms = 20000    # PICK 搜索超时
    self.approach_timeout_ms = 15000  # APPROACH 超时
    self.home_backoff_ms = 1500     # HOME 离线倒车超时

    # ——— IMU 融合 ———
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
    self.imu_mag_calib_min_samples = 50
    self.imu_mag_calib_min_span = 80.0
    # 关磁靠墙: 少28°→1.084; +6°→1.103; CCW少10°→1.135
    self.imu_gyro_scale = 1.135
    self.imu_spin_beta = 0.01
    self.imu_spin_dps = 40.0

  def hdg_off_for(self, cls_id):
    i = int(cls_id)
    if 0 <= i < len(self.hdg_off):
      return float(self.hdg_off[i])
    return 0.0

  # ——— 序列化（JSON key 全中文）——————————————————————

  def to_dict(self):
    return {
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
      "最终接近速度": float(self.tracking.final_approach_speed),
      "搜索速度": float(self.tracking.search_speed),
      "目标类别": int(self.tracking.target_class),
      "最低置信度": int(self.tracking.min_confidence),
      "确认帧数": int(self.tracking.confirm_frames),
      "丢失帧数": int(self.tracking.lost_frames),
      "停止位置": float(self.tracking.stop_bottom_pct),
      "绕行起始位置": float(self.tracking.stage_bottom_pct),
      "接触位置": float(self.tracking.contact_bottom_pct),
      "视觉执行极性": float(self.tracking.bearing_actuation_sign),
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
      "比赛模式": str(self.match_mode),
      "场心航向": float(self.push_hdg_ref),
      "推箱偏角": [float(x) for x in self.hdg_off],
      "搜索顺序": [int(x) for x in self.match_order],
      "严格目标": bool(self.strict_target),
      "单车目标类别": int(self.single_target_class),
      "行驶占空比": float(self.drive_duty),
      "推箱占空比": float(self.push_duty),
      "航向执行极性": float(self.yaw_actuation_sign),
      "绕行速度": float(self.orbit_speed),
      "绕行方向": float(self.orbit_direction_sign),
      "绕行距离P": float(self.orbit_radial_kp),
      "绕行距离上限": float(self.orbit_radial_max),
      "绕行超时": int(self.orbit_timeout_ms),
      "绕行航向容差": float(self.orbit_yaw_tol_deg),
      "绕行居中容差": float(self.orbit_center_tol_pct),
      "绕行距离容差": float(self.orbit_range_tol_pct),
      "绕行确认帧数": int(self.orbit_confirm_frames),
      "绕行丢失帧数": int(self.orbit_lost_frames),
      "最终接近超时": int(self.final_approach_timeout_ms),
      "绕物总超时": int(self.approach_cluster_timeout_ms),
      "行驶超时": int(self.drive_timeout_ms),
      "推箱超时": int(self.push_timeout_ms),
      "清线时间": int(self.push_clear_ms),
      "推箱监护帧数": int(self.push_watch_frames),
      "推箱左容差": float(self.push_cx_left_min),
      "推箱右容差": float(self.push_cx_right_max),
      "推箱纠偏占空比": float(self.push_correct_duty),
      "推箱丢失盲区": int(self.push_lost_blind_ms),
      "掉头超时": int(self.next_spin_ms),
      "后退超时": int(self.recover_backoff_ms),
      "回库超时": int(self.home_timeout_ms),
      "航向容差": float(self.align_tol_deg),
      "调试输出": bool(self.debug_output),
      # TCS
      "黄线确认帧数": int(self.tcs_confirm_n),
      "黄线R下限": float(self.tcs_r_min),
      "黄线G下限": float(self.tcs_g_min),
      "黄线B上限": float(self.tcs_b_max),
      "黄线亮度下限": int(self.tcs_c_min),
      # 超时
      "搜索超时": int(self.pick_timeout_ms),
      "接近超时": int(self.approach_timeout_ms),
      "离线超时": int(self.home_backoff_ms),
      # IMU
      "标定采样数": int(self.imu_calibrate_samples),
      "滤波增益": float(self.imu_beta),
      "陀螺静止阈值": float(self.imu_gyro_still),
      "加计静止阈值": float(self.imu_acc_still),
      "零偏校正速率": float(self.imu_bias_alpha),
      "磁融合速率": float(self.imu_mag_alpha),
      "磁融合死区": float(self.imu_mag_dead),
      "磁融合上限": float(self.imu_mag_pull_max),
      "磁融合静止帧数": int(self.imu_mag_still_need),
      "零偏静止帧数": int(self.imu_still_needed),
      "磁参考LPFa": float(self.imu_mag_lpf_alpha),
      "磁标定最少样本": int(self.imu_mag_calib_min_samples),
      "磁标定最小跨度": float(self.imu_mag_calib_min_span),
      "陀螺刻度": float(self.imu_gyro_scale),
      "转动滤波增益": float(self.imu_spin_beta),
      "转动角速度阈": float(self.imu_spin_dps),
    }

  def _apply_dict(self, d):
    for k, v in d.items():
      if k.startswith("//") or k.startswith("__"):
        continue
      try:
        self._set_one(k, v)
      except Exception as e:
        info("CONFIG", "skip '%s': %s" % (k, e))

  def _set_one(self, k, v):
    # 航向 PID
    if k == "航向P":
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
    elif k == "最终接近速度":
      self.tracking.final_approach_speed = float(v)
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
    elif k == "绕行起始位置":
      self.tracking.stage_bottom_pct = float(v)
    elif k == "接触位置":
      self.tracking.contact_bottom_pct = float(v)
    elif k == "视觉执行极性":
      self.tracking.bearing_actuation_sign = float(v)
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
    elif k == "比赛模式":
      v = str(v).strip().lower()
      self.match_mode = "pre" if v in ("pre", "预赛") else "final"
    elif k == "场心航向":
      self.push_hdg_ref = float(v)
    elif k == "推箱偏角":
      self.hdg_off = [float(x) for x in v]
    elif k == "搜索顺序":
      self.match_order = [int(x) for x in v]
    elif k == "严格目标":
      self.strict_target = bool(v)
    elif k == "单车目标类别":
      self.single_target_class = int(v)
    elif k == "行驶占空比":
      self.drive_duty = float(v)
    elif k == "推箱占空比":
      self.push_duty = float(v)
    elif k == "航向执行极性":
      self.yaw_actuation_sign = float(v)
    elif k == "绕行速度":
      self.orbit_speed = float(v)
    elif k == "绕行方向":
      self.orbit_direction_sign = float(v)
    elif k == "绕行距离P":
      self.orbit_radial_kp = float(v)
    elif k == "绕行距离上限":
      self.orbit_radial_max = float(v)
    elif k == "绕行超时":
      self.orbit_timeout_ms = int(v)
    elif k == "绕行航向容差":
      self.orbit_yaw_tol_deg = float(v)
    elif k == "绕行居中容差":
      self.orbit_center_tol_pct = float(v)
    elif k == "绕行距离容差":
      self.orbit_range_tol_pct = float(v)
    elif k == "绕行确认帧数":
      self.orbit_confirm_frames = int(v)
    elif k == "绕行丢失帧数":
      self.orbit_lost_frames = int(v)
    elif k == "最终接近超时":
      self.final_approach_timeout_ms = int(v)
    elif k == "绕物总超时":
      self.approach_cluster_timeout_ms = int(v)
    elif k == "行驶超时":
      self.drive_timeout_ms = int(v)
    elif k == "推箱超时":
      self.push_timeout_ms = int(v)
    elif k == "清线时间":
      self.push_clear_ms = int(v)
    elif k == "推箱监护帧数":
      self.push_watch_frames = int(v)
    elif k == "推箱左容差":
      self.push_cx_left_min = float(v)
    elif k == "推箱右容差":
      self.push_cx_right_max = float(v)
    elif k == "推箱纠偏占空比":
      self.push_correct_duty = float(v)
    elif k == "推箱丢失盲区":
      self.push_lost_blind_ms = int(v)
    elif k == "掉头超时":
      self.next_spin_ms = int(v)
    elif k == "后退超时":
      self.recover_backoff_ms = int(v)
    elif k == "回库超时":
      self.home_timeout_ms = int(v)
    elif k == "航向容差":
      self.align_tol_deg = float(v)
    elif k == "调试输出":
      self.debug_output = bool(v)
    # TCS
    elif k == "黄线确认帧数": self.tcs_confirm_n = int(v)
    elif k == "黄线R下限": self.tcs_r_min = float(v)
    elif k == "黄线G下限": self.tcs_g_min = float(v)
    elif k == "黄线B上限": self.tcs_b_max = float(v)
    elif k == "黄线亮度下限": self.tcs_c_min = int(v)
    # 超时
    elif k == "搜索超时": self.pick_timeout_ms = int(v)
    elif k == "接近超时": self.approach_timeout_ms = int(v)
    elif k == "离线超时": self.home_backoff_ms = int(v)
    # IMU
    elif k == "标定采样数": self.imu_calibrate_samples = int(v)
    elif k == "滤波增益": self.imu_beta = float(v)
    elif k == "陀螺静止阈值": self.imu_gyro_still = float(v)
    elif k == "加计静止阈值": self.imu_acc_still = float(v)
    elif k == "零偏校正速率": self.imu_bias_alpha = float(v)
    elif k == "磁融合速率": self.imu_mag_alpha = float(v)
    elif k == "磁融合死区": self.imu_mag_dead = float(v)
    elif k == "磁融合上限": self.imu_mag_pull_max = float(v)
    elif k == "磁融合静止帧数": self.imu_mag_still_need = int(v)
    elif k == "零偏静止帧数": self.imu_still_needed = int(v)
    elif k == "磁参考LPFa": self.imu_mag_lpf_alpha = float(v)
    elif k == "磁标定最少样本": self.imu_mag_calib_min_samples = int(v)
    elif k == "磁标定最小跨度": self.imu_mag_calib_min_span = float(v)
    elif k == "陀螺刻度": self.imu_gyro_scale = float(v)
    elif k == "转动滤波增益": self.imu_spin_beta = float(v)
    elif k == "转动角速度阈": self.imu_spin_dps = float(v)

  @classmethod
  def load(cls, path=CONFIG_FILE):
    cfg = cls()
    try:
      with open(path, "r") as f:
        d = json.load(f)
        if isinstance(d, dict):
          cfg._apply_dict(d)
        else:
          info("CONFIG", "invalid JSON, reset to default")
          cfg.save(path)
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


def save_config():
  config.save()
