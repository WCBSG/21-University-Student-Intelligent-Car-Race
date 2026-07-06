"""
ObjectTracker.py — 视觉目标追踪器

基于 OpenART Plus 摄像头检测结果，自主导航到目标前方。

状态机:
  IDLE → SEARCHING → TRACKING → COMPLETE → IDLE

控制策略:
  - Bearing PI: bbox 中心偏移 → 旋转修正（复用 HeadingPID）
  - 恒定低速前进（不估计距离）
  - bbox 底边触底 (y2 ≥ 95%) → 停车
  - 丢失目标后反转 30° 回头确认

依赖: CameraReceiver, MotionControl, ImuSensor, HeadingPID

用法:
  tracker = ObjectTracker(motors, imu, camera, cfg)
  tracker.start()                    # 开始追踪
  # 在主循环中 (仅 camera.update() 返回 True 时):
  tracker.update()
"""

import math
from time import ticks_ms, ticks_diff
from Motor import MotionControl
from HeadingController import HeadingPID

# =============================================================================
#                         ObjectTracker 主类
# =============================================================================

class ObjectTracker:
  """
  视觉目标追踪器。

  公有 API:
    start() / stop() / update() / update_pid_gains()
    state / target_info
  """

  # ——————————————————————————————————————————————————————————
  #                      初始化
  # ——————————————————————————————————————————————————————————

  def __init__(self, motors, imu, camera, config_dict):
    """
    参数:
      motors:      MotionControl 实例
      imu:         ImuSensor 实例
      camera:      CameraReceiver 实例
      config_dict: config 字典（持久化参数）
    """
    self._motors = motors
    self._imu = imu
    self._camera = camera
    self._cfg = config_dict

    # 状态
    self._state = 'IDLE'          # IDLE | SEARCHING | TRACKING | COMPLETE
    self._confirm_count = 0       # 连续新帧计数（有目标+/无目标-共用）
    self._prev_has_target = False # 上一帧是否有目标

    # Bearing PI（复用 HeadingPID）
    self._bearing_pid = HeadingPID(
      kp=self._cfg.get('trk_bearing_kp', 1.5),
      ki=self._cfg.get('trk_bearing_ki', 0.05),
      kd=self._cfg.get('trk_bearing_kd', 0.0),
      max_output=self._cfg.get('trk_bearing_max', 60.0),
      deadband=self._cfg.get('trk_bearing_db', 0.02)
    )
    self._last_control_ms = 0

    # 搜索
    self._search_direction = 1     # 1=顺时针, -1=逆时针

    # 反转回退
    self._reverse_target = None    # 反转目标 yaw (rad)，None=不处于反转
    self._reverse_start_yaw = 0.0
    self._reverse_accumulated = 0.0

    # 目标信息（供菜单显示）
    self._current_target = None    # (cls_id, score, cx, cy, w, h, area)
    self._target_info_str = ""

  # ——————————————————————————————————————————————————————————
  #                      公有 API
  # ——————————————————————————————————————————————————————————

  def start(self):
    """
    开始视觉追踪。始终从 SEARCHING 状态开始（旋转寻找目标）。
    返回 False 如果 IMU 未校准。
    """
    if not self._imu.is_calibrated:
      return False

    if self._state == 'TRACKING':
      return True

    self._state = 'SEARCHING'
    self._confirm_count = 0
    self._prev_has_target = False
    self._reverse_target = None
    self._search_direction = 1
    self._current_target = None
    self._bearing_pid.reset()
    return True

  def stop(self):
    """立即停止：刹车 → IDLE。"""
    self._state = 'IDLE'
    self._motors.stop()
    self._confirm_count = 0
    self._reverse_target = None
    self._current_target = None
    self._target_info_str = ""
    if self._camera:
      self._camera.flush()

  def update(self):
    """
    状态机调度。每主循环迭代调用。
    """
    if self._state == 'IDLE' or self._state == 'COMPLETE':
      return

    if self._state == 'SEARCHING':
      self._update_searching()
      return
    elif self._state == 'TRACKING':
      self._update_tracking()
    elif self._state == 'TRACKING':
      self._update_tracking()

  def update_pid_gains(self):
    """从 config 热重载 PID 参数"""
    self._bearing_pid.set_gains(
      kp=self._cfg.get('trk_bearing_kp', 1.5),
      ki=self._cfg.get('trk_bearing_ki', 0.05),
      kd=self._cfg.get('trk_bearing_kd', 0.0),
    )
    self._bearing_pid.max_output = self._cfg.get('trk_bearing_max', 60.0)
    self._bearing_pid.deadband = self._cfg.get('trk_bearing_db', 0.02)

  @property
  def state(self):
    return self._state

  @property
  def target_info(self):
    return self._target_info_str

  # ——————————————————————————————————————————————————————————
  #                      搜索状态
  # ——————————————————————————————————————————————————————————

  def _update_searching(self):
    """每帧（有新数据时）搜索逻辑"""
    dets = self._camera.get_detections()
    target = self._select_target(dets)
    confirm_frames = int(self._cfg.get('trk_confirm_frames', 20))

    if self._reverse_target is not None:
      # ——— 反转回退阶段 ———
      current_yaw = self._imu.get_yaw()

      # 检查是否找到目标
      if target is not None:
        self._reverse_target = None
        self._confirm_count = 0
        self._prev_has_target = False
        self._enter_tracking(target)
        return

      # 累积反转角度
      delta = self._normalize_angle(current_yaw - self._reverse_start_yaw)
      self._reverse_start_yaw = current_yaw
      self._reverse_accumulated += abs(delta)

      rev_angle = self._cfg.get('trk_reverse_angle', 30.0)
      if self._reverse_accumulated >= rev_angle:
        # 反转完成 → 正常搜索
        self._reverse_target = None
        self._reverse_accumulated = 0.0

      # 驱动（使用已翻转的搜索方向）
      speed = self._cfg.get('trk_search_speed', 15.0)
      s = speed * self._search_direction
      self._motors.setSpeed([s, s, s])
      return

    # ——— 正常搜索 ———
    if target is not None:
      self._confirm_count += 1
      if self._confirm_count >= confirm_frames:
        self._enter_tracking(target)
        return
    else:
      self._confirm_count = 0

    # 旋转搜索
    speed = self._cfg.get('trk_search_speed', 15.0)
    s = speed * self._search_direction
    self._motors.setSpeed([s, s, s])

  # ——————————————————————————————————————————————————————————
  #                      追踪状态
  # ——————————————————————————————————————————————————————————

  def _update_tracking(self):
    """每帧（有新数据时）追踪逻辑"""
    dets = self._camera.get_detections()
    target = self._select_target(dets)
    confirm_frames = int(self._cfg.get('trk_confirm_frames', 4))

    if target is not None:
      # 有目标：重置丢失计数，执行追踪
      self._confirm_count = 0
      self._prev_has_target = True
      self._current_target = target
      self._target_info_str = self._fmt_target(target)

      cx, y2 = target[6], target[9]

      # 检查到达条件：bbox 底边触底
      stop_pct = self._cfg.get('trk_stop_bottom_pct', 95.0)
      if y2 >= stop_pct:
        self._enter_complete()
        return

      # Bearing PI 修正
      bearing_error = (cx - 50.0) / 50.0  # 归一化 [-1, 1]
      now = ticks_ms()
      dt = ticks_diff(now, self._last_control_ms) / 1000.0
      if dt <= 0.0 or dt > 0.5:
        dt = 0.1  # 默认 ~100ms（10fps 相机）
      self._last_control_ms = now

      rotation = -self._bearing_pid.update(bearing_error, dt)

      # 前进 + 旋转
      forward_speed = self._cfg.get('trk_approach_speed', 15.0)
      self._drive(forward_speed, rotation)

    else:
      # 无目标：累积丢失计数
      self._confirm_count += 1
      self._prev_has_target = False

      if self._confirm_count >= confirm_frames:
        # 丢失确认 → 反转回退
        self._enter_searching_reverse()
        return

      # 短暂丢失期间：保持当前运动（减速滑行）
      # 不更新 PID，电机保持上帧状态（已在无新帧时自然不变）

  # ——————————————————————————————————————————————————————————
  #                      到达状态
  # ——————————————————————————————————————————————————————————

  def _enter_complete(self):
    """切换到 COMPLETE 状态"""
    self._state = 'COMPLETE'
    self._motors.stop()
    self._bearing_pid.reset()
    self._reverse_target = None
    self._confirm_count = 0
    self._target_info_str = "COMPLETE"

  # ——————————————————————————————————————————————————————————
  #                      状态切换辅助
  # ——————————————————————————————————————————————————————————

  def _enter_tracking(self, target):
    """进入 TRACKING 状态"""
    self._state = 'TRACKING'
    self._current_target = target
    self._target_info_str = self._fmt_target(target)
    self._confirm_count = 0
    self._prev_has_target = True
    self._reverse_target = None
    self._bearing_pid.reset()
    self._last_control_ms = ticks_ms()

  def _enter_searching_reverse(self):
    """进入 SEARCHING 状态 + 设置反转回退"""
    self._state = 'SEARCHING'
    self._current_target = None
    self._target_info_str = ""
    self._confirm_count = 0
    self._prev_has_target = False
    self._bearing_pid.reset()

    # 设置反转目标
    rev_angle = self._cfg.get('trk_reverse_angle', 30.0)
    current_yaw = self._imu.get_yaw()
    self._reverse_target = current_yaw - self._search_direction * rev_angle
    self._reverse_start_yaw = current_yaw
    self._reverse_accumulated = 0.0
    self._search_direction *= -1  # 反转搜索方向

  # ——————————————————————————————————————————————————————————
  #                      目标选择
  # ——————————————————————————————————————————————————————————

  def _select_target(self, detections):
    """
    从检测列表中选出最佳目标。

    规则:
      1. 过滤: 按指定类别 (trk_target_class, 255=任意) + 最低置信度
      2. 排序: 优先指定类别 > 最大面积
    """
    if not detections:
      return None

    target_class = int(self._cfg.get('trk_target_class', 7))
    min_conf = self._cfg.get('trk_min_confidence', 22)

    candidates = []
    for d in detections:
      cls_id, score = d[0], d[1]
      # 类别过滤 (7=任意)
      if target_class != 7 and cls_id != target_class:
        continue
      # 置信度过滤 (0-31)
      if score < min_conf:
        continue
      # 宽高过滤
      if d[4] <= 0 or d[5] <= 0:
        continue
      candidates.append(d)  # 预计算元组: (..., cx, cy, area, y2)

    if not candidates:
      return None

    # 排序: 指定类别优先 → 最大面积
    candidates.sort(key=lambda x: (
      0 if x[0] == target_class or target_class == 7 else 1,
      -x[8]
    ))
    return candidates[0]

  # ——————————————————————————————————————————————————————————
  #                      驱动
  # ——————————————————————————————————————————————————————————

  def _drive(self, forward_speed, rotation):
    """
    全向轮驱动: 前进 + 旋转。

    forward_speed: 前进占空比 (%)
    rotation:      旋转修正占空比 (正值=顺时针)
    """
    forward = MotionControl.move(forward_speed, 0.0)
    duties = [
      self._clamp(forward[0] + rotation, -100.0, 100.0),
      self._clamp(forward[1] + rotation, -100.0, 100.0),
      self._clamp(forward[2] + rotation, -100.0, 100.0),
    ]
    self._motors.setSpeed(duties)

  # ——————————————————————————————————————————————————————————
  #                      工具函数
  # ——————————————————————————————————————————————————————————

  @staticmethod
  def _normalize_angle(angle):
    """角度归一化到 [-180, 180]"""
    while angle > 180.0:
      angle -= 360.0
    while angle < -180.0:
      angle += 360.0
    return angle

  @staticmethod
  def _clamp(val, lo, hi):
    if val < lo: return lo
    if val > hi: return hi
    return val

  @staticmethod
  def _fmt_target(target):
    """格式化目标信息供菜单显示"""
    if target is None:
      return "--"
    cls_id, area = target[0], target[8]
    return "cls=%d area=%.0f%%" % (int(cls_id), area / 100.0)
