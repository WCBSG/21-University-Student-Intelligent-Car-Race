"""
ctrl/heading_mode.py — HDG Mode（直线 / 锁航）

Mode.update 不申请转移；由 FSM handle(GO_STRAIGHT|LOCK_YAW) 配置后 enter。
"""

from time import ticks_ms, ticks_diff
from Motor import MotionControl
from HeadingController import HeadingPID
from app.mode import Mode, HDG


class HeadingMode(Mode):
  id = HDG  # Arbiter owner

  def __init__(self, arbiter, imu, cfg):
    self._arb = arbiter
    self._imu = imu
    self._cfg = cfg
    self._pid = HeadingPID(gains=cfg.heading)
    self._sub = "straight"   # 'straight' | 'lock'
    self._target = 0.0
    self._speed = 50.0
    self._last_ms = ticks_ms()
    # 供菜单显示
    self.last_error = 0.0

  def configure(self, sub, target=None, speed=None):
    """FSM handle 在 transition 前调用。"""
    self._sub = sub
    if speed is not None:
      self._speed = float(speed)
    else:
      self._speed = float(self._cfg.target_speed)
    if target is not None:
      self._target = self._normalize(float(target))
    else:
      self._target = None  # enter 时取当前 yaw

  def enter(self):
    self._pid.reset()
    self._last_ms = ticks_ms()
    if self._sub == "straight":
      self._target = self._imu.get_yaw()
    elif self._target is None:
      self._target = self._imu.get_yaw()

  def exit(self):
    self._pid.reset()
    self.last_error = 0.0

  def update(self, dt, sensors):
    if not self._imu.is_calibrated:
      return
    now = ticks_ms()
    real_dt = ticks_diff(now, self._last_ms) / 1000.0
    if real_dt <= 0.0 or real_dt > 0.5:
      real_dt = dt if dt > 0 else 0.02
    self._last_ms = now

    current = self._imu.get_yaw()
    error = self._normalize(self._target - current)
    self.last_error = error
    correction = self._pid.update(error, real_dt)
    rotation = -correction

    if self._sub == "straight":
      forward = MotionControl.move(self._speed, 0.0)
      duties = [
        self._clamp(forward[0] + rotation, -100.0, 100.0),
        self._clamp(forward[1] + rotation, -100.0, 100.0),
        self._clamp(forward[2] + rotation, -100.0, 100.0),
      ]
    else:
      duties = [rotation, rotation, rotation]

    self._arb.write(self.id, duties)

  @property
  def target_heading(self):
    return self._target if self._target is not None else 0.0

  @property
  def forward_speed(self):
    return self._speed if self._sub == "straight" else 0.0

  @property
  def sub_mode(self):
    return self._sub

  @staticmethod
  def _normalize(angle):
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
