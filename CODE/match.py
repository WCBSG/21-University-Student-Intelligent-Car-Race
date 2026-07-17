# match.py — MatchRunner 核心 + HOME（ISR/Hunt 为 mixin，分文件编译）
from time import ticks_ms, ticks_diff, ticks_add
from log import info
from motion import MotionControl, HeadingPID
from config import CLS_LEFT, CLS_RIGHT
from match_isr import MatchIsr, wrap as _wrap
from match_hunt import MatchHunt

ABORT = "ABORT"
STOP = "STOP"

_MATCH_PHASES = ("PICK", "APPROACH", "ORBIT", "FINAL_APPROACH", "PUSH", "BACKOFF")


class MatchRunner(MatchIsr, MatchHunt):
  OWNER = "MATCH"

  def __init__(self, robot, arbiter, tcs, cfg):
    self._robot = robot
    self._arb = arbiter
    self._tcs = tcs
    self._cfg = cfg
    self.phase = "IDLE"
    self.scored_count = 0
    self._sub = ""
    self._phase_ms = 0
    self._see_streak = 0
    self._remaining = []
    self._active_cls = None
    self._yaw_target = 0.0
    self._hold_yaw = 0.0
    self._home_y2 = None
    self._home_deadline = 0
    self._hdg_pid = HeadingPID(gains=cfg.heading)
    self._bearing_pid = HeadingPID(gains=cfg.tracking_bearing)
    self._hdg_ms = ticks_ms()
    self._ctrl_ms = ticks_ms()
    self._orbit_confirm = 0
    self._vision_lost = 0
    self._approach_deadline = 0
    self._cmd_forward = 0.0
    self._cmd_lateral = 0.0
    self._cmd_rotation = 0.0
    self.fault_reason = ""
    self._push_bad = 0
    self._push_bad_kind = ""
    self._boundary_armed = False
    self._boundary_pending = False
    self._backoff_busy = False
    self._backoff_sub = "IDLE"
    self._backoff_ms = 0
    self._want_home = False
    self._post_backoff = None
    self._def_score = 0
    self._def_bo = 0
    self._def_armed = False
    self._bo_retreat = [0.0, 0.0, 0.0]
    self._bo_spin = [0.0, 0.0, 0.0]

  @property
  def backoff_busy(self):
    return self._backoff_busy

  @property
  def field_lock_enabled(self):
    return self.phase in ("PICK", "APPROACH", "ORBIT", "FINAL_APPROACH", "PUSH")

  @property
  def stage(self):
    p = self.phase
    if p == "IDLE":
      return "INIT"
    if p == "LEAVE":
      return "LEAVE"
    if p == "HOME":
      return "HOME"
    if p in ("DONE", "FAULT"):
      return p
    if p in _MATCH_PHASES or self._backoff_busy:
      return "MATCH"
    return p

  def start(self):
    if self.phase not in ("IDLE", "DONE"):
      info("MATCH", "cannot start, phase=%s" % self.phase)
      return False
    self.scored_count = 0
    c = self._cfg
    strict_cls = int(getattr(c, "single_target_class", -1))
    if getattr(c, "strict_target", False) and 0 <= strict_cls <= 2:
      self._remaining = [strict_cls]
    else:
      self._remaining = [int(x) for x in c.match_order]
    self._active_cls = None
    self.fault_reason = ""
    self._approach_deadline = 0
    self._see_streak = 0
    self._sub = ""
    self._phase_ms = ticks_ms()
    c.tracking.target_class = 7
    c.match_allow = None
    self._hold_yaw = self._yaw()
    self._hdg_pid.reset()
    self._hdg_ms = ticks_ms()
    self._boundary_armed = False
    self._boundary_pending = False
    self._backoff_busy = False
    self._backoff_sub = "IDLE"
    self._want_home = False
    self._post_backoff = None
    self._def_score = 0
    self._def_bo = 0
    self._def_armed = False
    self._cache_backoff_duties()
    self.phase = "LEAVE"
    self._take_motors(abort=True)
    info("MATCH", "START → LEAVE hold_yaw=%.1f" % self._hold_yaw)
    return True

  def stop(self):
    info("MATCH", "STOP")
    self._backoff_busy = False
    self._backoff_sub = "IDLE"
    self._post_backoff = None
    self._want_home = False
    self.phase = "IDLE"
    self._sub = ""
    self._cfg.match_allow = None
    self._cfg.tracking.target_class = 7
    self._robot.handle(ABORT)
    self._brake()

  def tick(self, dt, sensors):
    self.flush_deferred()
    if self.phase in ("IDLE", "DONE", "FAULT"):
      return
    if self._post_backoff:
      action = self._post_backoff
      self._post_backoff = None
      if action == "HOME":
        self._enter_home()
      else:
        self._enter_pick()
      return
    if self._backoff_busy:
      return
    if self.phase == "LEAVE":
      self._tick_leave(sensors)
    elif self.phase == "PICK":
      self._tick_pick(sensors)
    elif self.phase == "APPROACH":
      self._tick_approach(sensors)
    elif self.phase == "ORBIT":
      self._tick_orbit(sensors)
    elif self.phase == "FINAL_APPROACH":
      self._tick_final_approach(sensors)
    elif self.phase == "PUSH":
      self._tick_push(sensors)
    elif self.phase == "HOME":
      self._tick_home(sensors)

  def _yaw(self):
    return self._robot._imu.get_yaw()

  def _yaw_err(self, target):
    return _wrap(target - self._yaw())

  def _take_motors(self, abort=False):
    self._robot.handle(ABORT if abort else STOP)
    self._arb.acquire(self.OWNER)

  def _on_line(self, sensors):
    if sensors and sensors.get("tcs_on_line"):
      return True
    return bool(self._tcs.on_line)

  def _write_move(self, speed, angle=0.0):
    if self._backoff_busy:
      return
    self._set_command(speed, 0.0, 0.0)
    self._arb.write(self.OWNER, MotionControl.move(speed, angle))

  def _set_command(self, forward, lateral, rotation):
    self._cmd_forward = float(forward)
    self._cmd_lateral = float(lateral)
    self._cmd_rotation = float(rotation)

  def _write_move_locked(self, speed, yaw_tgt):
    if self._backoff_busy:
      return
    now = ticks_ms()
    dt = ticks_diff(now, self._hdg_ms) / 1000.0
    if dt <= 0.0 or dt > 0.5:
      dt = 0.02
    self._hdg_ms = now
    err = self._yaw_err(yaw_tgt)
    rot = self._cfg.yaw_actuation_sign * self._hdg_pid.update(err, dt)
    fwd = MotionControl.move(float(speed), 0.0)
    duties = [
      self._clamp(fwd[0] + rot, -100.0, 100.0),
      self._clamp(fwd[1] + rot, -100.0, 100.0),
      self._clamp(fwd[2] + rot, -100.0, 100.0),
    ]
    self._set_command(speed, 0.0, rot)
    self._arb.write(self.OWNER, duties)

  def _control_dt(self):
    now = ticks_ms()
    dt = ticks_diff(now, self._ctrl_ms) / 1000.0
    if dt <= 0.0 or dt > 0.5:
      dt = 0.05
    self._ctrl_ms = now
    return dt

  def _write_vector(self, forward, lateral, rot):
    if self._backoff_busy:
      return
    fwd = MotionControl.move(float(forward), 0.0)
    if lateral >= 0.0:
      side = MotionControl.move(float(lateral), 90.0)
    else:
      side = MotionControl.move(float(-lateral), -90.0)
    duties = [
      self._clamp(fwd[i] + side[i] + rot, -100.0, 100.0)
      for i in range(3)
    ]
    self._set_command(forward, lateral, rot)
    self._arb.write(self.OWNER, duties)

  @staticmethod
  def _clamp(v, lo, hi):
    if v < lo:
      return lo
    if v > hi:
      return hi
    return v

  def _write_spin(self, duty):
    if self._backoff_busy:
      return
    d = float(duty)
    self._set_command(0.0, 0.0, d)
    self._arb.write(self.OWNER, [d, d, d])

  def _hold_brake(self):
    self._set_command(0.0, 0.0, 0.0)
    self._arb.hold_brake(self.OWNER)

  def _brake(self):
    self._set_command(0.0, 0.0, 0.0)
    self._arb.force_brake()

  def _spin_toward(self, target):
    err = self._yaw_err(target)
    if abs(err) <= float(self._cfg.align_tol_deg):
      self._hold_brake()
      return True
    s = float(self._cfg.drive_duty)
    s = self._cfg.yaw_actuation_sign * (s if err > 0 else -s)
    self._write_spin(s)
    return False

  def _fault(self, why):
    info("MATCH", "FAULT: %s" % why)
    self.fault_reason = str(why)
    self._backoff_busy = False
    self._post_backoff = None
    self._robot.handle(ABORT)
    self._brake()
    self.phase = "FAULT"
    self._sub = ""

  def _skip_or_home(self, why):
    info("MATCH", "%s → skip cls=%s" % (why, self._active_cls))
    if self._active_cls is not None and self._active_cls in self._remaining:
      self._remaining.remove(self._active_cls)
    elif self._remaining:
      self._remaining.pop(0)
    if self._remaining:
      self._enter_pick()
    else:
      self._fault("%s; no target class left (%d/%d scored)" % (
        why, self.scored_count, int(self._cfg.match_target_count)))

  def _seen_target(self, sensors, need=4):
    if sensors and sensors.get("new_frame"):
      if sensors.get("has_target"):
        self._see_streak += 1
      else:
        self._see_streak = 0
    return self._see_streak >= need

  def _set_pick_class(self):
    c = self._cfg
    strict_cls = int(getattr(c, "single_target_class", -1))
    if getattr(c, "strict_target", False) and 0 <= strict_cls <= 2:
      c.match_allow = None
      c.tracking.target_class = strict_cls
      self._active_cls = strict_cls
      return
    if c.match_mode == "pre" or not self._remaining:
      c.tracking.target_class = 7
      c.match_allow = None
      self._active_cls = None
    else:
      c.match_allow = list(self._remaining)
      c.tracking.target_class = int(self._remaining[0])
      self._active_cls = int(self._remaining[0])

  def _push_yaw(self):
    c = self._cfg
    if c.match_mode == "pre":
      return None
    cls = self._active_cls
    if cls is None:
      cls = c.tracking.target_class
    off = c.hdg_off_for(cls)
    if off == 0.0 and all(abs(float(x)) < 1e-6 for x in c.hdg_off):
      return None
    return _wrap(c.push_hdg_ref + off)

  def _home_plan(self):
    c = self._cfg
    href = float(c.push_hdg_ref)
    layout = int(c.start_layout)
    left = c.hdg_off_for(CLS_LEFT)
    right = c.hdg_off_for(CLS_RIGHT)
    if layout == 2:
      return _wrap(href + left), _wrap(href + 180.0)
    if layout == 3:
      return _wrap(href + right), _wrap(href + 180.0)
    if layout == 4:
      return _wrap(href + left), None
    return _wrap(href + 180.0), None

  def _enter_home(self):
    y1, y2 = self._home_plan()
    self._yaw_target = y1
    self._home_y2 = y2
    self._home_deadline = ticks_add(ticks_ms(), int(self._cfg.home_timeout_ms))
    self._boundary_armed = False
    self._boundary_pending = False
    self._take_motors()
    self.phase = "HOME"
    if self._tcs.on_line:
      self._sub = "LEAVE_LINE"
    else:
      self._sub = "LEG1_TURN"
    self._phase_ms = ticks_ms()
    info("MATCH", "→ HOME sub=%s" % self._sub)

  def _tick_home(self, sensors):
    now = ticks_ms()
    if ticks_diff(now, self._home_deadline) > 0:
      self._fault("HOME timeout — gate not confirmed")
      return
    on_line = self._on_line(sensors)
    if self._sub == "LEAVE_LINE":
      if not on_line:
        self._sub = "LEG1_TURN"
        self._phase_ms = now
        info("MATCH", "HOME → LEG1_TURN")
        return
      self._write_move(-float(self._cfg.drive_duty), 0.0)
      return
    if self._sub == "LEG1_TURN":
      if self._spin_toward(self._yaw_target):
        self._hold_yaw = self._yaw_target
        self._hdg_pid.reset()
        self._sub = "LEG1_DRIVE"
        self._phase_ms = now
        self._tcs.reset_crossed()
        info("MATCH", "HOME → LEG1_DRIVE")
      return
    if self._sub == "LEG1_DRIVE":
      if on_line:
        if self._home_y2 is None:
          self._brake()
          self._finish()
          return
        self._hold_brake()
        self._sub = "BACKOFF"
        self._phase_ms = now
        info("MATCH", "HOME → BACKOFF")
        return
      if abs(self._yaw_err(self._hold_yaw)) > 12.0:
        self._spin_toward(self._hold_yaw)
        return
      self._write_move_locked(float(self._cfg.drive_duty), self._hold_yaw)
      return
    if self._sub == "BACKOFF":
      if not on_line or ticks_diff(now, self._phase_ms) > int(self._cfg.home_backoff_ms):
        self._hold_brake()
        self._sub = "BACKOFF_TURN"
        self._phase_ms = now
        info("MATCH", "HOME → BACKOFF turn")
        return
      self._write_move(-float(self._cfg.drive_duty), 0.0)
      return
    if self._sub == "BACKOFF_TURN":
      if self._spin_toward(self._home_y2):
        self._hold_yaw = self._home_y2
        self._hdg_pid.reset()
        self._sub = "LEG2_DRIVE"
        self._phase_ms = now
        self._tcs.reset_crossed()
        info("MATCH", "HOME → LEG2_DRIVE")
      return
    if self._sub == "LEG2_DRIVE":
      if on_line:
        self._finish()
        return
      if abs(self._yaw_err(self._hold_yaw)) > 12.0:
        self._spin_toward(self._hold_yaw)
        return
      self._write_move_locked(float(self._cfg.drive_duty), self._hold_yaw)

  def _finish(self):
    self._brake()
    self._robot.handle(STOP)
    self.phase = "DONE"
    self._sub = ""
    info("MATCH", "DONE scored=%d" % self.scored_count)

  @property
  def is_running(self):
    return self.phase not in ("IDLE", "DONE", "FAULT")

  def navigation_snapshot(self, sensors=None):
    target = sensors.get("target") if sensors else None
    cx = float(target[6]) if target is not None else -1.0
    y2 = float(target[9]) if target is not None else -1.0
    target_yaw = self._yaw_target
    if self.phase == "LEAVE" or self.phase == "PUSH":
      target_yaw = self._hold_yaw
    elif self.phase == "BACKOFF" and self._backoff_sub == "SPIN":
      target_yaw = self._yaw_target
    yaw = self._yaw()
    return (
      self.phase, self._sub, yaw, target_yaw, _wrap(target_yaw - yaw),
      cx, y2, self._cmd_forward, self._cmd_lateral, self._cmd_rotation,
      self._orbit_confirm, self._vision_lost)

  @property
  def info(self):
    if self._sub:
      return "Match:%s/%s scored=%d" % (self.phase, self._sub, self.scored_count)
    return "Match:%s scored=%d" % (self.phase, self.scored_count)
