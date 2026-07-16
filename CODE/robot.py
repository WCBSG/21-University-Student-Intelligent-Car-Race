"""
robot.py — Robot FSM + Track modes
IDLE | SEARCH | TRACK | COMPLETE | FAULT
"""

from time import ticks_ms, ticks_diff
from log import info
from motion import MotionControl, HeadingPID


# =============================================================================
#                          Intent 常量
# =============================================================================

ABORT   = "ABORT"
STOP    = "STOP"
START_TRACK = "START_TRACK"


# =============================================================================
#                          状态常量 + Debouncer + Mode
# =============================================================================

IDLE     = "IDLE"
SEARCH   = "SEARCH"
TRACK    = "TRACK"
COMPLETE = "COMPLETE"
FAULT    = "FAULT"


class Debouncer:
  """连续满足 condition 达 threshold 次 → ready。"""
  def __init__(self, threshold):
    self._n = 0
    self._thr = int(threshold)

  def reset(self):
    self._n = 0

  def tick(self, condition):
    self._n = self._n + 1 if condition else 0

  def ready(self):
    return self._n >= self._thr


class Mode:
  id = "mode"
  def enter(self): pass
  def update(self, dt, sensors): pass
  def exit(self): pass


class IdleMode(Mode):
  id = "idle"
  def __init__(self, arbiter):
    self._arb = arbiter
  def enter(self):
    self._arb.force_brake()


# =============================================================================
#                         Intent 转移表
# =============================================================================

_INTENT_TABLE = {
  (START_TRACK, IDLE):     SEARCH,
  (START_TRACK, SEARCH):   None,
  (START_TRACK, TRACK):    None,
  (START_TRACK, COMPLETE): None,
  (START_TRACK, FAULT):    None,

  (STOP, IDLE):     None,
  (STOP, SEARCH):   IDLE,
  (STOP, TRACK):    IDLE,
  (STOP, COMPLETE): IDLE,
  (STOP, FAULT):    IDLE,

  (ABORT, IDLE):     IDLE,
  (ABORT, SEARCH):   IDLE,
  (ABORT, TRACK):    IDLE,
  (ABORT, COMPLETE): IDLE,
  (ABORT, FAULT):    IDLE,
}


# =============================================================================
#                              RobotFSM
# =============================================================================

class RobotFSM:
  def __init__(self, arbiter, cfg, imu, modes=None):
    self._arb   = arbiter
    self._cfg   = cfg
    self._imu   = imu
    self.state  = IDLE
    self._modes = modes if modes is not None else {}
    if IDLE not in self._modes:
      self._modes[IDLE] = IdleMode(arbiter)
    self._mode   = self._modes[IDLE]
    self._confirm = Debouncer(cfg.tracking.confirm_frames)
    self._lost    = Debouncer(cfg.tracking.lost_frames)
    self.search_phase = "spin"

  @property
  def mode(self):
    return self._mode

  def set_mode_map(self, modes):
    self._modes.update(modes)

  def transition(self, new_state):
    if new_state == self.state and new_state != IDLE:
      return
    old = self._mode
    if old is not None:
      old.exit()
      self._arb.release(old.id)
    info("FSM", "%s → %s" % (self.state, new_state))
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

  # ——— handle —————————————————————

  def handle(self, intent, arg=None):
    if intent == START_TRACK and not self._imu.is_calibrated:
      return False

    nxt = _INTENT_TABLE.get((intent, self.state))
    if nxt is None:
      return nxt is not None  # START_TRACK on same state → silently do nothing (not error)

    if intent in (ABORT, STOP):
      self.transition(IDLE)
      self._arb.force_brake()
      return True

    if nxt != self.state:
      self.transition(nxt)
    return True

  # ——— tick ——————————————————————————————

  def tick(self, dt, sensors):
    if sensors is None:
      sensors = {}
    if sensors.get("cam_timeout") and self.state in (SEARCH, TRACK):
      self.transition(FAULT)
      return
    if sensors.get("new_frame"):
      self.on_camera_frame(
        sensors.get("has_target", False),
        sensors.get("y2", 0.0))
    if self._mode is not None:
      self._mode.update(dt, sensors)

  # ——— 相机帧转移 —————————————————————————

  def on_camera_frame(self, has_target, y2=0.0):
    stop_pct = self._cfg.tracking.stop_bottom_pct
    # 决赛需先停在目标前方绕到正确推送侧；预赛仍直接接近接触位置。
    if getattr(self._cfg, "match_mode", "final") != "pre":
      stop_pct = self._cfg.tracking.stage_bottom_pct
    if self.state == SEARCH:
      self._confirm.tick(has_target)
      if self._confirm.ready():
        self.transition(TRACK)
    elif self.state == TRACK:
      if has_target and y2 >= stop_pct:
        self.transition(COMPLETE)
        return
      self._lost.tick(not has_target)
      if self._lost.ready():
        search = self._modes.get(SEARCH)
        if search is not None and hasattr(search, "begin_reverse"):
          search.begin_reverse()
        else:
          self.search_phase = "reverse"
        self.transition(SEARCH)


# =============================================================================
#                           Track 模式
# =============================================================================

class TrackSearchMode(Mode):
  id = SEARCH

  def __init__(self, arbiter, imu, cfg, robot_ref):
    self._arb = arbiter; self._imu = imu; self._cfg = cfg; self._robot = robot_ref
    self._direction = 1; self._rev_start_yaw = 0.0; self._rev_acc = 0.0

  def enter(self):
    self._rev_acc = 0.0
    self._rev_start_yaw = self._imu.get_yaw()
    if self._robot.search_phase == "reverse": self._direction *= -1

  def exit(self): pass

  def begin_reverse(self):
    self._robot.search_phase = "reverse"

  def update(self, dt, sensors):
    if sensors.get("has_target") and self._robot.search_phase != "reverse":
      self._arb.write(self.id, [0, 0, 0])
      return
    if self._robot.search_phase == "reverse":
      yaw = self._imu.get_yaw()
      d = yaw - self._rev_start_yaw
      while d > 180.0: d -= 360.0
      while d < -180.0: d += 360.0
      self._rev_acc += abs(d)
      self._rev_start_yaw = yaw
      if self._rev_acc >= self._cfg.tracking.reverse_angle:
        self._robot.search_phase = "spin"; self._rev_acc = 0.0
    s = self._cfg.tracking.search_speed * self._direction
    self._arb.write(self.id, [s, s, s])


class TrackApproachMode(Mode):
  id = TRACK

  def __init__(self, arbiter, imu, cfg):
    self._arb = arbiter; self._imu = imu; self._cfg = cfg
    self._pid = HeadingPID(gains=cfg.tracking_bearing)
    self._last_ms = ticks_ms()

  def enter(self): self._pid.reset(); self._last_ms = ticks_ms()
  def exit(self): self._pid.reset()

  def update(self, dt, sensors):
    t = sensors.get("target")
    if t is None:
      self._arb.write(self.id, [0, 0, 0])  # 目标丢失 → 刹车，避免沿用旧 PWM
      return
    be = (t[6] - 50.0) / 50.0
    now = ticks_ms()
    real_dt = ticks_diff(now, self._last_ms) / 1000.0
    if real_dt <= 0.0 or real_dt > 0.5: real_dt = 0.1
    self._last_ms = now
    # 实车标定：目标在右侧(be>0)时需要三轮正修正，使车顺时针右转。
    rot = self._cfg.tracking.bearing_actuation_sign * self._pid.update(be, real_dt)
    fwd = MotionControl.move(self._cfg.tracking.approach_speed, 0.0)
    self._arb.write(self.id, [max(-100, min(100, fwd[i] + rot)) for i in range(3)])


class CompleteMode(Mode):
  id = COMPLETE
  def __init__(self, arbiter): self._arb = arbiter
  def enter(self): self._arb.write(self.id, [0, 0, 0])
  def update(self, dt, sensors): pass


class FaultMode(Mode):
  id = FAULT
  def __init__(self, arbiter): self._arb = arbiter
  def enter(self): self._arb.force_brake()
  def update(self, dt, sensors): pass


# =============================================================================
#                              工厂
# =============================================================================

def build_robot(arbiter, cfg, imu):
  robot = RobotFSM(arbiter, cfg, imu)
  robot.set_mode_map({
    IDLE:     IdleMode(arbiter),
    SEARCH:   TrackSearchMode(arbiter, imu, cfg, robot),
    TRACK:    TrackApproachMode(arbiter, imu, cfg),
    COMPLETE: CompleteMode(arbiter),
    FAULT:    FaultMode(arbiter),
  })
  robot._mode = robot._modes[IDLE]
  return robot


