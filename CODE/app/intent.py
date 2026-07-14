"""
app/intent.py — Intent 枚举 + 队列（ABORT 优先）

每拍 drain 全部；若含 ABORT 则只执行 ABORT，清空其余。
"""

# Intent 名称（MicroPython 无 Enum 时用字符串常量）
ABORT = "ABORT"
STOP = "STOP"
START_TRACK = "START_TRACK"
GO_STRAIGHT = "GO_STRAIGHT"
LOCK_YAW = "LOCK_YAW"
RECONNECT = "RECONNECT"


class IntentQueue:
  """主循环每拍 drain()；post() 由按键/菜单调用。"""

  def __init__(self):
    self._q = []

  def post(self, intent, arg=None):
    self._q.append((intent, arg))

  def clear(self):
    self._q = []

  def drain(self):
    """
    返回本拍应处理的 Intent 列表（已按 ABORT 规则折叠）。
    若存在 ABORT：仅返回 [(ABORT, None)]。
    否则按 FIFO 返回全部并清空。
    """
    if not self._q:
      return []
    has_abort = False
    for intent, _arg in self._q:
      if intent == ABORT:
        has_abort = True
        break
    if has_abort:
      self._q = []
      return [(ABORT, None)]
    out = self._q
    self._q = []
    return out

  def __len__(self):
    return len(self._q)
