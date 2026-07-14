"""
app/fsm.py — Robot FSM（步 4：完整 tick / 相机帧转移）

状态: IDLE | HDG | SEARCH | TRACK | COMPLETE | FAULT
Mode.update → None；转移仅由 handle / tick / on_camera_frame 发起。
"""

from app import intent as I
from app.mode import (
  IDLE, HDG, SEARCH, TRACK, COMPLETE, FAULT, ALL_STATES,
  Debouncer, Mode, IdleMode,
)


# 再导出，供外部 from app.fsm import SEARCH 等
__all__ = [
  "IDLE", "HDG", "SEARCH", "TRACK", "COMPLETE", "FAULT", "ALL_STATES",
  "Debouncer", "Mode", "IdleMode", "RobotFSM", "build_robot",
]


_INTENT_TABLE = {
  (I.GO_STRAIGHT, IDLE): HDG,
  (I.GO_STRAIGHT, HDG): HDG,
  (I.GO_STRAIGHT, COMPLETE): HDG,
  (I.GO_STRAIGHT, FAULT): None,
  (I.GO_STRAIGHT, SEARCH): None,
  (I.GO_STRAIGHT, TRACK): None,
  (I.LOCK_YAW, IDLE): HDG,
  (I.LOCK_YAW, HDG): HDG,
  (I.LOCK_YAW, SEARCH): None,
  (I.LOCK_YAW, TRACK): None,
  (I.LOCK_YAW, COMPLETE): None,
  (I.LOCK_YAW, FAULT): None,
  (I.START_TRACK, IDLE): SEARCH,
  (I.START_TRACK, HDG): SEARCH,
  (I.START_TRACK, SEARCH): None,
  (I.START_TRACK, TRACK): None,
  (I.START_TRACK, COMPLETE): None,
  (I.START_TRACK, FAULT): None,
  (I.STOP, IDLE): None,
  (I.STOP, HDG): IDLE,
  (I.STOP, SEARCH): IDLE,
  (I.STOP, TRACK): IDLE,
  (I.STOP, COMPLETE): IDLE,
  (I.STOP, FAULT): IDLE,
  (I.ABORT, IDLE): IDLE,
  (I.ABORT, HDG): IDLE,
  (I.ABORT, SEARCH): IDLE,
  (I.ABORT, TRACK): IDLE,
  (I.ABORT, COMPLETE): IDLE,
  (I.ABORT, FAULT): IDLE,
  (I.RECONNECT, IDLE): IDLE,
  (I.RECONNECT, FAULT): IDLE,
  (I.RECONNECT, HDG): None,
  (I.RECONNECT, SEARCH): None,
  (I.RECONNECT, TRACK): None,
  (I.RECONNECT, COMPLETE): None,
}


class RobotFSM:
  def __init__(self, arbiter, cfg, imu, modes=None):
    self._arb = arbiter
    self._cfg = cfg
    self._imu = imu
    self.state = IDLE
    self._modes = modes if modes is not None else {}
    if IDLE not in self._modes:
      self._modes[IDLE] = IdleMode(arbiter)
    self._mode = self._modes[IDLE]
    self._confirm = Debouncer(cfg.tracking.confirm_frames)
    self._lost = Debouncer(cfg.tracking.lost_frames)
    self.search_phase = "spin"
    self.reconnect_pending = False
    self.target_info = ""

  @property
  def mode(self):
    return self._mode

  def set_mode_map(self, modes):
    self._modes.update(modes)

  def transition(self, new_state):
    if new_state == self.state and new_state != IDLE:
      return
    old_state = self.state
    old = self._mode
    if old is not None:
      old.exit()
      self._arb.release(old.id)
    print("[FSM] %s → %s" % (old_state, new_state))
    self.state = new_state
    self._mode = self._modes.get(new_state)
    if self._mode is None:
      self._mode = Mode()
      self._mode.id = new_state
    self._arb.acquire(self._mode.id)
    self._mode.enter()
    self._confirm.reset()
    self._lost.reset()
    if new_state != SEARCH:
      self.search_phase = "spin"

  def handle(self, intent, arg=None):
    if intent == I.START_TRACK:
      if not self._imu.is_calibrated:
        return False

    key = (intent, self.state)
    if key not in _INTENT_TABLE:
      return False
    nxt = _INTENT_TABLE[key]
    if nxt is None:
      return False

    if intent == I.RECONNECT:
      self.reconnect_pending = True
      if self.state == FAULT:
        self.transition(IDLE)
      return True

    if intent in (I.ABORT, I.STOP):
      self.reconnect_pending = False
      self.transition(IDLE)
      self._arb.force_brake()
      self.target_info = ""
      return True

    # 配置 HDG Mode
    if intent == I.GO_STRAIGHT and nxt == HDG:
      hdg_mode = self._modes.get(HDG)
      if hdg_mode is not None and hasattr(hdg_mode, "configure"):
        hdg_mode.configure("straight", speed=arg)
    elif intent == I.LOCK_YAW and nxt == HDG:
      hdg_mode = self._modes.get(HDG)
      if hdg_mode is not None and hasattr(hdg_mode, "configure"):
        hdg_mode.configure("lock", target=arg)

    if nxt != self.state:
      self.transition(nxt)
    elif intent in (I.GO_STRAIGHT, I.LOCK_YAW) and self.state == HDG:
      if self._mode is not None:
        self._mode.exit()
        self._mode.enter()
    return True

  def drain_and_handle(self, intent_queue):
    for intent, arg in intent_queue.drain():
      self.handle(intent, arg)

  def tick(self, dt, sensors):
    if sensors is None:
      sensors = {}

    # 仅运动态超时进 FAULT；IDLE/COMPLETE 菜单空闲不停发也不踢故障
    if sensors.get("cam_timeout") and self.state in (HDG, SEARCH, TRACK):
      self.transition(FAULT)
      return

    # 有新相机帧时做去抖转移（也可由 main 先调 on_camera_frame）
    if sensors.get("new_frame"):
      self.on_camera_frame(
        sensors.get("has_target", False),
        sensors.get("y2", 0.0),
      )

    if self._mode is not None:
      self._mode.update(dt, sensors)

    # 同步显示信息
    m = self._mode
    if self.state == TRACK and hasattr(m, "target_info"):
      self.target_info = m.target_info
    elif self.state == COMPLETE:
      self.target_info = "COMPLETE"
    elif self.state not in (TRACK, COMPLETE):
      if self.state == SEARCH:
        pass
      else:
        self.target_info = ""

  def on_camera_frame(self, has_target, y2=0.0):
    """按相机帧调用（或由 tick 在 new_frame 时调用）。"""
    stop_pct = self._cfg.tracking.stop_bottom_pct

    if self.state == SEARCH:
      self._confirm.tick(has_target)
      if has_target and self._confirm._n > 0:
        print("[FSM] SEARCH confirm=%d/%d" % (self._confirm._n, self._confirm._thr))
      if self._confirm.ready():
        self.transition(TRACK)

    elif self.state == TRACK:
      if has_target and y2 >= stop_pct:
        print("[FSM] TRACK y2=%.1f >= %.1f → COMPLETE" % (y2, stop_pct))
        self.transition(COMPLETE)
        return
      self._lost.tick(not has_target)
      if not has_target and self._lost._n > 0:
        print("[FSM] TRACK lost=%d/%d" % (self._lost._n, self._lost._thr))
      if self._lost.ready():
        print("[FSM] TRACK lost confirmed → SEARCH+reverse")
        search = self._modes.get(SEARCH)
        if search is not None and hasattr(search, "begin_reverse"):
          search.begin_reverse()
        else:
          self.search_phase = "reverse"
        self.transition(SEARCH)


def build_robot(arbiter, cfg, imu):
  """工厂：注册全部 Mode。"""
  from ctrl.heading_mode import HeadingMode
  from ctrl.track import TrackSearchMode, TrackApproachMode, CompleteMode, FaultMode

  robot = RobotFSM(arbiter, cfg, imu)
  hdg = HeadingMode(arbiter, imu, cfg)
  search = TrackSearchMode(arbiter, imu, cfg, robot)
  approach = TrackApproachMode(arbiter, imu, cfg)
  complete = CompleteMode(arbiter)
  fault = FaultMode(arbiter)
  robot.set_mode_map({
    IDLE: IdleMode(arbiter),
    HDG: hdg,
    SEARCH: search,
    TRACK: approach,
    COMPLETE: complete,
    FAULT: fault,
  })
  robot._mode = robot._modes[IDLE]
  return robot
