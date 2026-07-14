"""
ctrl/match.py — 比赛 Mode：PUSH(推出) / RETURN(回库)

PUSH: TRACK 到达(y2≥95%)后继续直推，直到物体推出场外
RETURN: 180°调头 + 航向锁定直行回发车区
"""

from time import ticks_ms, ticks_diff
from Motor import MotionControl
from HeadingController import HeadingPID
from app.mode import Mode, PUSH, RETURN


class PushMode(Mode):
  """直推模式：保持前进方向，持续固定时间/距离推出物体。"""

  id = PUSH

  def __init__(self, arbiter, cfg):
    self._arb = arbiter
    self._cfg = cfg
    self._start_ms = 0
    self._push_ms = 1500  # 推出持续时间 ms（可配置）

  def enter(self):
    self._start_ms = ticks_ms()

  def exit(self):
    pass

  def update(self, dt, sensors):
    elapsed = ticks_diff(ticks_ms(), self._start_ms)
    if elapsed >= self._push_ms:
      # 推完 → 停转。转移由 FSM 计时或出界判定触发
      self._arb.write(self.id, [0, 0, 0])
      return

    speed = self._cfg.tracking.approach_speed
    forward = MotionControl.move(speed, 0.0)
    duties = [self._clamp(forward[0], -100, 100),
              self._clamp(forward[1], -100, 100),
              self._clamp(forward[2], -100, 100)]
    self._arb.write(self.id, duties)

  @property
  def push_done(self):
    return ticks_diff(ticks_ms(), self._start_ms) >= self._push_ms

  @staticmethod
  def _clamp(val, lo, hi):
    if val < lo: return lo
    if val > hi: return hi
    return val


class ReturnMode(Mode):
  """
  回库模式：调头 180° + 航向锁定直行。
  复用 HeadingPID + Motor.move 与 HDG straight 相同逻辑。
  """

  id = RETURN

  def __init__(self, arbiter, imu, cfg):
    self._arb = arbiter
    self._imu = imu
    self._cfg = cfg
    self._pid = HeadingPID(gains=cfg.heading)
    self._target = 0.0
    self._speed = 0.0
    self._last_ms = 0

  def enter(self):
    # 目标航向 = 当前 + 180°（调头）
    current = self._imu.get_yaw()
    self._target = self._norm(current + 180.0)
    self._speed = self._cfg.target_speed
    self._pid.reset()
    self._last_ms = ticks_ms()

  def exit(self):
    self._pid.reset()

  def update(self, dt, sensors):
    if not self._imu.is_calibrated:
      return

    now = ticks_ms()
    real_dt = ticks_diff(now, self._last_ms) / 1000.0
    if real_dt <= 0.0 or real_dt > 0.5:
      real_dt = 0.02
    self._last_ms = now

    error = self._norm(self._target - self._imu.get_yaw())
    correction = self._pid.update(error, real_dt)
    rotation = -correction

    forward = MotionControl.move(self._speed, 0.0)
    duties = [
      self._clamp(forward[0] + rotation, -100, 100),
      self._clamp(forward[1] + rotation, -100, 100),
      self._clamp(forward[2] + rotation, -100, 100),
    ]
    self._arb.write(self.id, duties)

  @staticmethod
  def _norm(angle):
    while angle > 180.0: angle -= 360.0
    while angle < -180.0: angle += 360.0
    return angle

  @staticmethod
  def _clamp(val, lo, hi):
    if val < lo: return lo
    if val > hi: return hi
    return val
