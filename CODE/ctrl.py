"""
ctrl.py — 电机仲裁 + 航向PID + 视觉跟踪模式（三合一）
"""

from time import ticks_ms, ticks_diff
from Motor import MotionControl
from fsm import Mode, SEARCH, TRACK, COMPLETE, FAULT


# =============================================================================
#                          MotorArbiter — 电机唯一写入口
# =============================================================================

class MotorArbiter:
  def __init__(self, motors):
    self._motors = motors
    self._owner = None
    self._d0 = self._d1 = self._d2 = 0.0
    self._last_owner_warn_ms = 0

  @property
  def owner(self): return self._owner

  @property
  def motors_active(self):
    return abs(self._d0) > 1.0 or abs(self._d1) > 1.0 or abs(self._d2) > 1.0

  def acquire(self, cid):
    if self._owner is not None and self._owner != cid:
      self._motors.brake()
      self._d0 = self._d1 = self._d2 = 0.0
    self._owner = cid

  def release(self, cid):
    if self._owner == cid:
      self._motors.brake()
      self._d0 = self._d1 = self._d2 = 0.0
      self._owner = None

  def write(self, cid, duties):
    if self._owner == cid:
      self._motors.setSpeed(duties)
      self._d0, self._d1, self._d2 = float(duties[0]), float(duties[1]), float(duties[2])
      return True
    now = ticks_ms()
    if ticks_diff(now, self._last_owner_warn_ms) >= 1000:
      self._last_owner_warn_ms = now
      print("[ARB] reject write owner=%s caller=%s" % (self._owner, cid))
    return False

  def hold_brake(self, cid):
    if self._owner == cid:
      self._motors.brake()
      self._d0 = self._d1 = self._d2 = 0.0

  def force_brake(self):
    self._motors.brake()
    self._d0 = self._d1 = self._d2 = 0.0
    self._owner = None


# =============================================================================
#                             HeadingPID — 航向 PID
# =============================================================================

class HeadingPID:
  def __init__(self, kp=2.0, ki=0.0, kd=0.0, max_output=100.0, deadband=0.0,
               gains=None):
    self._g = gains
    self.kp = kp; self.ki = ki; self.kd = kd
    self.max_output = max_output; self.deadband = deadband
    self._integral = 0.0; self._prev_error = 0.0; self._first_update = True

  def _params(self):
    g = self._g
    if g is not None:
      return g.kp, g.ki, g.kd, g.max_out, g.deadband
    return self.kp, self.ki, self.kd, self.max_output, self.deadband

  def update(self, error, dt):
    kp, ki, kd, mx, db = self._params()
    if abs(error) < db: error = 0.0
    if self._first_update:
      self._prev_error = error; self._first_update = False
      return 0.0
    self._integral += error * dt
    d = (error - self._prev_error) / dt if dt > 1e-6 else 0.0
    out = kp * error + ki * self._integral + kd * d
    if out > mx:
      out = mx
      if ki > 0 and error * self._integral > 0: self._integral -= error * dt
    elif out < -mx:
      out = -mx
      if ki > 0 and error * self._integral > 0: self._integral -= error * dt
    self._prev_error = error
    return out

  def reset(self):
    self._integral = 0.0; self._prev_error = 0.0; self._first_update = True


# =============================================================================
#                           select_target — 目标筛选
# =============================================================================

def select_target(detections, cfg):
  if not detections: return None
  tc, mc = int(cfg.tracking.target_class), cfg.tracking.min_confidence
  allow = getattr(cfg, "match_allow", None)
  candidates = []
  for d in detections:
    cid, sc = d[0], d[1]
    if allow is not None:
      if cid not in allow: continue
    elif tc != 7 and cid != tc: continue
    if sc < mc: continue
    if d[4] <= 0 or d[5] <= 0: continue
    candidates.append(d)
  if not candidates: return None
  candidates.sort(key=lambda x: (0 if x[0] == tc or tc == 7 else 1, -x[8]))
  return candidates[0]


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
      self._rev_acc += abs(_norm(yaw - self._rev_start_yaw))
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
    rot = -self._pid.update(be, real_dt)
    fwd = MotionControl.move(self._cfg.tracking.approach_speed, 0.0)
    self._arb.write(self.id, [_clamp(fwd[i] + rot, -100, 100) for i in range(3)])


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
#                              工具函数
# =============================================================================

def _norm(a):
  while a > 180.0: a -= 360.0
  while a < -180.0: a += 360.0
  return a

def _clamp(v, lo, hi):
  if v < lo: return lo
  if v > hi: return hi
  return v
