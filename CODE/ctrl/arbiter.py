"""
ctrl/arbiter.py — 电机唯一写入口

同一时刻仅一个 owner 可写。切换/ABORT 时 brake（非 coast）。
"""


class MotorArbiter:
  """
  acquire(id)  — 抢占写权；若已有其他 owner 先 brake
  release(id)  — 释放并 brake
  write(id, duties) — 仅 owner 可写，否则静默丢弃
  motors_active — 最近一次有效 duty 是否在转（供 mag 门控；[0,0,0] 为停）
  """

  def __init__(self, motors):
    self._motors = motors
    self._owner = None
    self._d0 = 0.0
    self._d1 = 0.0
    self._d2 = 0.0

  @property
  def owner(self):
    return self._owner

  @property
  def motors_active(self):
    """|duty| > 1% 视为在转（停转写零仍可开 mag）。"""
    return (abs(self._d0) > 1.0 or abs(self._d1) > 1.0 or abs(self._d2) > 1.0)

  def _set_duties(self, duties):
    self._d0 = float(duties[0])
    self._d1 = float(duties[1])
    self._d2 = float(duties[2])

  def _clear_duties(self):
    self._d0 = 0.0
    self._d1 = 0.0
    self._d2 = 0.0

  def acquire(self, controller_id):
    if self._owner is not None and self._owner != controller_id:
      self._motors.brake()
      self._clear_duties()
    self._owner = controller_id

  def release(self, controller_id):
    if self._owner == controller_id:
      self._motors.brake()
      self._clear_duties()
      self._owner = None

  def write(self, controller_id, duties):
    if self._owner == controller_id:
      self._motors.setSpeed(duties)
      self._set_duties(duties)

  def hold_brake(self, controller_id):
    """电子刹车但保留 owner（对齐后停稳，不 inert 滑行）。"""
    if self._owner == controller_id:
      self._motors.brake()
      self._clear_duties()

  def force_brake(self):
    """ABORT：无条件刹车并清空 owner。"""
    self._motors.brake()
    self._clear_duties()
    self._owner = None