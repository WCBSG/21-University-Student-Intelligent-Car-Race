"""
ctrl/arbiter.py — 电机唯一写入口

同一时刻仅一个 owner 可写。切换/ABORT 时 brake（非 coast）。
"""


class MotorArbiter:
  """
  acquire(id)  — 抢占写权；若已有其他 owner 先 brake
  release(id)  — 释放并 brake
  write(id, duties) — 仅 owner 可写，否则静默丢弃
  """

  def __init__(self, motors):
    self._motors = motors
    self._owner = None

  @property
  def owner(self):
    return self._owner

  def acquire(self, controller_id):
    if self._owner is not None and self._owner != controller_id:
      self._motors.brake()
    self._owner = controller_id

  def release(self, controller_id):
    if self._owner == controller_id:
      self._motors.brake()
      self._owner = None

  def write(self, controller_id, duties):
    if self._owner == controller_id:
      self._motors.setSpeed(duties)

  def force_brake(self):
    """ABORT：无条件刹车并清空 owner。"""
    self._motors.brake()
    self._owner = None
