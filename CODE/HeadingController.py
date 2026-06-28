"""
HeadingController.py — 航向角闭环控制

基于 IMU 偏航角反馈的 PID 控制器，支持两种模式：
  1. 直线模式 (straight): 锁定初始航向，直线前进
  2. 锁定模式 (lock): 原地旋转到目标航向并保持

依赖: MotionControl (Motor.py), ImuSensor (imu.py), config

用法:
  hdg = HeadingController(motors, imu, config.config)
  hdg.mode_straight(speed=50)   # 锁定当前航向，50% 速度前进
  hdg.mode_lock(target=90)      # 原地旋转到 90°
  hdg.mode_idle()               # 停止控制
  # 在主循环: hdg.update()
"""

import math
from time import ticks_ms, ticks_diff
from Motor import MotionControl


# =============================================================================
#                         增量式 PID 控制器
# =============================================================================

class HeadingPID:
  """
  航向 PID，带死区和积分抗饱和（back-calculation）。

  参数:
    kp, ki, kd  — PID 增益
    max_output   — 输出限幅 (对称)
    deadband     — 死区 (deg)，|error| < deadband 时输出 0
  """

  def __init__(self, kp=2.0, ki=0.0, kd=0.0, max_output=100.0, deadband=0.0):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.max_output = max_output
    self.deadband = deadband

    self._integral = 0.0
    self._prev_error = 0.0
    self._first_update = True

  # ——————————————————————————————————————————————————————————
  #                      核心更新
  # ——————————————————————————————————————————————————————————

  def update(self, error, dt):
    """
    输入:
      error — 角度误差 (deg)，已归一化到 [-180, 180]
      dt    — 距上次调用的时间间隔 (s)

    返回: PID 输出 (在 [-max_output, max_output] 内)
    """
    # 死区
    if abs(error) < self.deadband:
      error = 0.0

    if self._first_update:
      self._prev_error = error
      self._first_update = False
      return 0.0

    # 积分项 (先累加，反算饱和时回退)
    self._integral += error * dt

    # 微分项 (on error)
    derivative = (error - self._prev_error) / dt if dt > 1e-6 else 0.0

    # PID 输出
    output = self.kp * error + self.ki * self._integral + self.kd * derivative

    # 输出限幅 + 积分反算抗饱和
    if output > self.max_output:
      output = self.max_output
      if self.ki > 0 and error * self._integral > 0:
        self._integral -= error * dt
    elif output < -self.max_output:
      output = -self.max_output
      if self.ki > 0 and error * self._integral < 0:
        self._integral -= error * dt

    self._prev_error = error
    return output

  # ——————————————————————————————————————————————————————————
  #                      参数管理
  # ——————————————————————————————————————————————————————————

  def reset(self):
    """重置积分和微分历史。"""
    self._integral = 0.0
    self._prev_error = 0.0
    self._first_update = True

  def set_gains(self, kp=None, ki=None, kd=None):
    """热更新 PID 增益（保留积分/微分状态）。"""
    if kp is not None: self.kp = kp
    if ki is not None: self.ki = ki
    if kd is not None: self.kd = kd


# =============================================================================
#                         航向闭环控制器
# =============================================================================

class HeadingController:
  """
  航向角闭环控制器。

  模式:
    'idle'     — 关闭，电机滑行
    'straight' — 直线模式：锁定初始航向 + 给定速度前进
    'lock'     — 锁定模式：原地旋转到目标航向

  全向轮运动学：
    - 前进分量 = MotionControl.move(speed, 0°) 的逆解
    - 旋转分量 = 同一值加到三个轮上 (同号同速 = 纯旋转)
    - 最终 duty = clamp(forward[i] + correction, -100, 100)
  """

  def __init__(self, motors, imu, config_dict):
    """
    参数:
      motors      — MotionControl 实例
      imu         — ImuSensor 实例
      config_dict — config 字典 (持久化参数)
    """
    self._motors = motors
    self._imu = imu
    self._config = config_dict

    self._mode = 'idle'
    self._target_heading = 0.0
    self._forward_speed = 0.0
    self._last_time = 0

    # 创建 PID
    self._pid = HeadingPID(
      kp=self._config.get('heading_kp', 2.0),
      ki=self._config.get('heading_ki', 0.0),
      kd=self._config.get('heading_kd', 0.0),
      max_output=self._config.get('heading_max_correction', 50.0),
      deadband=self._config.get('heading_deadband', 1.0)
    )

  # ——————————————————————————————————————————————————————————
  #                      模式切换
  # ——————————————————————————————————————————————————————————

  def mode_straight(self, speed=None):
    """
    直线模式：以当前航向为基准前进。
    若 speed 为 None，使用 config['target_speed']。
    """
    if not self._imu.is_calibrated:
      return False

    if speed is None:
      speed = self._config.get('target_speed', 50.0)

    self._mode = 'straight'
    self._target_heading = self._imu.get_yaw()
    self._forward_speed = float(speed)
    self._pid.reset()
    self._last_time = ticks_ms()
    return True

  def mode_lock(self, target=None):
    """
    锁定模式：原地旋转到目标航向。
    若 target 为 None，锁定当前航向。
    """
    if not self._imu.is_calibrated:
      return False

    if target is None:
      target = self._imu.get_yaw()

    self._mode = 'lock'
    self._target_heading = self._normalize_angle(float(target))
    self._forward_speed = 0.0
    self._pid.reset()
    self._last_time = ticks_ms()
    return True

  def mode_idle(self):
    """关闭航向控制，电机滑行停止。"""
    self._mode = 'idle'
    self._motors.stop()

  # ——————————————————————————————————————————————————————————
  #                      每帧更新
  # ——————————————————————————————————————————————————————————

  def update(self):
    """
    每帧调用（主循环 ~50Hz）。
    读取 IMU → 计算航向误差 → PID → 驱动电机。
    """
    if self._mode == 'idle':
      return

    if not self._imu.is_calibrated:
      return

    now = ticks_ms()
    dt = ticks_diff(now, self._last_time) / 1000.0

    # 防止 dt 异常（首次调用、溢出、长时间未调用）
    if dt <= 0.0:
      dt = 0.01
    elif dt > 0.5:
      dt = 0.01

    self._last_time = now

    current = self._imu.get_yaw()
    error = self._normalize_angle(self._target_heading - current)

    correction = self._pid.update(error, dt)

    # 方向修正：正 duty → 机器人顺时针 (yaw↓)，与 IMU yaw 正方向 (CCW↑) 相反
    # 故将 PID 修正量取反，使正误差 (target>current, 需CCW) → 负 duty → CCW 旋转
    rotation = -correction

    if self._mode == 'straight':
      # 前进分量 (全向轮逆运动学，angle=0 = 前方)
      forward = MotionControl.move(self._forward_speed, 0.0)
      duties = [
        self._clamp(forward[0] + rotation, -100.0, 100.0),
        self._clamp(forward[1] + rotation, -100.0, 100.0),
        self._clamp(forward[2] + rotation, -100.0, 100.0),
      ]
    elif self._mode == 'lock':
      # 纯旋转：三轮同速同向
      duties = [rotation, rotation, rotation]

    self._motors.setSpeed(duties)

  # ——————————————————————————————————————————————————————————
  #                      状态查询
  # ——————————————————————————————————————————————————————————

  @property
  def mode(self):
    """当前模式: 'idle' | 'straight' | 'lock'"""
    return self._mode

  @property
  def target_heading(self):
    """目标航向角 (deg)"""
    return self._target_heading

  @property
  def heading_error(self):
    """当前航向误差 (deg)，[-180, 180]"""
    if self._mode == 'idle' or not self._imu.is_calibrated:
      return 0.0
    return self._normalize_angle(self._target_heading - self._imu.get_yaw())

  @property
  def forward_speed(self):
    """直线模式下的前进速度"""
    return self._forward_speed

  # ——————————————————————————————————————————————————————————
  #                      PID 热更新
  # ——————————————————————————————————————————————————————————

  def update_pid_gains(self):
    """从 config 字典重新读取 PID 参数并应用（菜单调节后调用）。"""
    self._pid.set_gains(
      kp=self._config.get('heading_kp', 2.0),
      ki=self._config.get('heading_ki', 0.0),
      kd=self._config.get('heading_kd', 0.0),
    )
    self._pid.max_output = self._config.get('heading_max_correction', 50.0)
    self._pid.deadband = self._config.get('heading_deadband', 1.0)

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