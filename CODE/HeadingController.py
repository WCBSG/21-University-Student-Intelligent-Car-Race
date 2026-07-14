"""
HeadingController.py — 航向 PID（HeadingPID）

Mode 层（HeadingMode / TrackApproachMode）持有本 PID；
旧 HeadingController 类已删除以节省 RAM。
"""


class HeadingPID:
  """
  航向 PID，带死区和积分抗饱和（back-calculation）。

  优先持有 PidGains 引用（self._g），每次 update 读最新 kp/ki/kd。
  无引用时退回本地 kp/ki/kd（兼容旧构造）。
  """

  def __init__(self, kp=2.0, ki=0.0, kd=0.0, max_output=100.0, deadband=0.0,
               gains=None):
    self._g = gains
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.max_output = max_output
    self.deadband = deadband

    self._integral = 0.0
    self._prev_error = 0.0
    self._first_update = True

  def _params(self):
    g = self._g
    if g is not None:
      return g.kp, g.ki, g.kd, g.max_out, g.deadband
    return self.kp, self.ki, self.kd, self.max_output, self.deadband

  def update(self, error, dt):
    kp, ki, kd, max_output, deadband = self._params()

    if abs(error) < deadband:
      error = 0.0

    if self._first_update:
      self._prev_error = error
      self._first_update = False
      return 0.0

    self._integral += error * dt
    derivative = (error - self._prev_error) / dt if dt > 1e-6 else 0.0
    output = kp * error + ki * self._integral + kd * derivative

    if output > max_output:
      output = max_output
      if ki > 0 and error * self._integral > 0:
        self._integral -= error * dt
    elif output < -max_output:
      output = -max_output
      if ki > 0 and error * self._integral > 0:
        self._integral -= error * dt

    self._prev_error = error
    return output

  def reset(self):
    self._integral = 0.0
    self._prev_error = 0.0
    self._first_update = True

  def set_gains(self, kp=None, ki=None, kd=None):
    if self._g is not None:
      if kp is not None: self._g.kp = kp
      if ki is not None: self._g.ki = ki
      if kd is not None: self._g.kd = kd
      return
    if kp is not None: self.kp = kp
    if ki is not None: self.ki = ki
    if kd is not None: self.kd = kd
