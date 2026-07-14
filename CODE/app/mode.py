"""
app/mode.py — Mode 基类、Debouncer、状态常量（避免循环 import）
"""

IDLE = "IDLE"
HDG = "HDG"
SEARCH = "SEARCH"
TRACK = "TRACK"
COMPLETE = "COMPLETE"
FAULT = "FAULT"

ALL_STATES = (IDLE, HDG, SEARCH, TRACK, COMPLETE, FAULT)


class Debouncer:
  """连续满足 condition 达 threshold 次 → ready。按相机帧 tick。"""

  def __init__(self, threshold):
    self._n = 0
    self._thr = int(threshold) if threshold else 1

  def reset(self):
    self._n = 0

  def tick(self, condition):
    if condition:
      self._n += 1
    else:
      self._n = 0

  def ready(self):
    return self._n >= self._thr


class Mode:
  """Mode 接口。子类实现 enter/update/exit。不自行申请转移。"""

  id = "mode"

  def enter(self):
    pass

  def update(self, dt, sensors):
    pass

  def exit(self):
    pass


class IdleMode(Mode):
  id = "idle"

  def __init__(self, arbiter):
    self._arb = arbiter

  def enter(self):
    self._arb.force_brake()
