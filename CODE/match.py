# match.py — MatchRunner 核心 + HOME（ISR/Hunt 为 mixin，分文件编译）
# 主车：视觉/运动/计分/回库。单一 Match 状态机，全程占用电机。
from time import ticks_ms, ticks_diff, ticks_add
from log import info
from motion import MotionControl, HeadingPID, wrap_deg
from match_isr import MatchIsr
from match_hunt import MatchHunt

_MATCH_PHASES = ("HUNT", "ALIGN", "PUSH", "BACKOFF")


class MatchRunner(MatchIsr, MatchHunt):
  OWNER = "MATCH"

  def __init__(self, arbiter, tcs, cfg, imu):
    self._arb = arbiter
    self._tcs = tcs
    self._cfg = cfg
    self._imu = imu
    self.phase = "IDLE"
    self.scored_count = 0
    self._sub = ""
    self._phase_ms = 0
    self._see_streak = 0
    self._remaining = []
    self._active_cls = None
    # 视觉筛选（运行时状态，不写回 cfg）
    self._match_allow = None
    self._filter_class = int(cfg.tracking.target_class)
    self._yaw_target = 0.0
    self._hold_yaw = 0.0
    self._home_y2 = None
    self._home_deadline = 0
    self._hdg_pid = HeadingPID(gains=cfg.heading)
    self._bearing_pid = HeadingPID(gains=cfg.tracking_bearing)
    self._hdg_ms = ticks_ms()
    self._ctrl_ms = ticks_ms()
    self._track_ms = ticks_ms()
    self._orbit_confirm = 0
    self._vision_lost = 0
    self._confirm_n = 0
    self._lost_n = 0
    self._search_phase = "spin"
    self._search_dir = 1
    self._rev_acc = 0.0
    self._rev_start_yaw = 0.0
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
    self._yellow_hit = False
    self._yellow_hit_phase = ""
    self._field_entered = False       # 出库进场后才允许场锁 BACKOFF
    self._boundary_need_cross = False  # True=须先压黄线再离线才武装
    self._boundary_saw_line = False
    self._boundary_arm_ms = 0
    self._def_score = 0
    self._def_bo = 0
    self._def_armed = False
    self._bo_retreat = [0.0, 0.0, 0.0]
    self._bo_spin = [0.0, 0.0, 0.0]
    self._home_turn_ok = 0
    self._queue_miss = 0
    self._queue_see = 0
    self._queue_see_cls = -1
    self._spin_acc = 0.0

  @property
  def match_allow(self):
    return self._match_allow

  @property
  def filter_class(self):
    return self._filter_class

  @property
  def backoff_busy(self):
    return self._backoff_busy

  @property
  def field_lock_enabled(self):
    return self.phase in ("HUNT", "ALIGN", "PUSH")

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
    self._confirm_n = 0
    self._lost_n = 0
    self._search_phase = "spin"
    self._search_dir = 1
    self._sub = ""
    self._phase_ms = ticks_ms()
    self._filter_class = 7
    self._match_allow = None
    self._hold_yaw = self._yaw()
    self._hdg_pid.reset()
    self._hdg_ms = ticks_ms()
    self._boundary_armed = False
    self._boundary_pending = False
    self._backoff_busy = False
    self._backoff_sub = "IDLE"
    self._want_home = False
    self._post_backoff = None
    self._yellow_hit = False
    self._yellow_hit_phase = ""
    self._field_entered = False
    self._boundary_need_cross = False
    self._boundary_saw_line = False
    self._boundary_arm_ms = 0
    self._def_score = 0
    self._def_bo = 0
    self._def_armed = False
    self._queue_miss = 0
    self._queue_see = 0
    self._queue_see_cls = -1
    self._spin_acc = 0.0
    self._cache_backoff_duties()
    self.phase = "LEAVE"
    self._take_motors()
    info("MATCH", "START → LEAVE hold_yaw=%.1f" % self._hold_yaw)
    return True

  def stop(self):
    info("MATCH", "STOP")
    self._backoff_busy = False
    self._backoff_sub = "IDLE"
    self._post_backoff = None
    self._yellow_hit = False
    self._yellow_hit_phase = ""
    self._field_entered = False
    self._boundary_need_cross = False
    self._boundary_saw_line = False
    self._boundary_armed = False
    self._boundary_pending = False
    self._want_home = False
    self.phase = "IDLE"
    self._sub = ""
    self._match_allow = None
    self._filter_class = int(self._cfg.tracking.target_class)
    self._brake()

  def tick(self, dt, sensors):
    self.flush_deferred()
    if self.phase in ("IDLE", "DONE", "FAULT"):
      return
    # 黄线标志 → 主循环启动 BACKOFF（ISR 只急停）
    if self.consume_yellow_hit():
      return
    if self._post_backoff:
      action = self._post_backoff
      self._post_backoff = None
      if action == "HOME":
        self._enter_home()
      elif action == "FWD":
        self._enter_hunt(forward=True)
      else:
        self._enter_hunt()
      return
    if self._backoff_busy:
      self.step_backoff()
      return
    if self.phase == "LEAVE":
      self._tick_leave(sensors)
    elif self.phase == "HUNT":
      self._tick_hunt(sensors)
    elif self.phase == "ALIGN":
      self._tick_align(sensors)
    elif self.phase == "PUSH":
      self._tick_push(sensors)
    elif self.phase == "HOME":
      self._tick_home(sensors)

  def _yaw(self):
    return self._imu.get_yaw()

  def _yaw_err(self, target):
    return wrap_deg(target - self._yaw())

  def _take_motors(self, abort=False):
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

  def _write_orbit_front(self, spin, slip, forward=0.0):
    """绕车头前方轴转：spin+slip 同号 → M1/M2 慢、M3 快（见 test_orbit_axis）。
    满额 spin=40/slip=60 → 约 (20,20,80)。"""
    if self._backoff_busy:
      return
    spin = float(spin)
    slip = float(slip)
    if slip >= 0.0:
      side = MotionControl.move(slip, 90.0)
    else:
      side = MotionControl.move(-slip, -90.0)
    if abs(forward) > 1e-6:
      fwd = MotionControl.move(float(forward), 0.0)
    else:
      fwd = (0.0, 0.0, 0.0)
    duties = [
      self._clamp(fwd[i] + side[i] + spin, -100.0, 100.0)
      for i in range(3)
    ]
    self._set_command(forward, slip, spin)
    self._arb.write(self.OWNER, duties)

  def _hold_brake(self):
    self._set_command(0.0, 0.0, 0.0)
    self._arb.hold_brake(self.OWNER)

  def _brake(self):
    self._set_command(0.0, 0.0, 0.0)
    self._arb.force_brake()

  def _spin_toward(self, target):
    err = self._yaw_err(target)
    tol = float(self._cfg.align_tol_deg)
    if abs(err) <= tol:
      self._home_turn_ok += 1
      if self._home_turn_ok >= 3:
        self._hold_brake()
        self._home_turn_ok = 0
        return True
      self._hold_brake()
      return False
    self._home_turn_ok = 0
    # 近目标减速，避免 ±50 开环过冲乱转
    s = float(self._cfg.drive_duty)
    if abs(err) < 40.0:
      s = s * 0.45
    elif abs(err) < 80.0:
      s = s * 0.7
    s = self._cfg.yaw_actuation_sign * (s if err > 0 else -s)
    self._write_spin(s)
    return False

  def _fault(self, why):
    info("MATCH", "FAULT: %s" % why)
    self.fault_reason = str(why)
    self._backoff_busy = False
    self._post_backoff = None
    self._yellow_hit = False
    self._brake()
    self.phase = "FAULT"
    self._sub = ""

  def _skip_or_home(self, why):
    info("MATCH", "%s → skip cls=%s" % (why, self._active_cls))
    if self._active_cls is not None and self._active_cls in self._remaining:
      self._remaining.remove(self._active_cls)
      self._remaining.append(self._active_cls)
    if self._remaining:
      self._enter_hunt()
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
      self._match_allow = None
      self._filter_class = strict_cls
      self._active_cls = strict_cls
      return
    if self._remaining:
      self._match_allow = list(self._remaining)
      # 队首优先：select_target 按 filter_class 排序
      self._filter_class = int(self._remaining[0])
    else:
      self._match_allow = None
      self._filter_class = 7
    self._active_cls = None

  def _push_yaw(self):
    """决赛推箱车头朝向 = 场心 + 推箱偏角（物品应去的方向）。
    车须站在该方向反侧、朝该方向推；ALIGN 负责转到此朝向并贴近。
    """
    c = self._cfg
    if c.match_mode == "pre":
      return None
    cls = self._active_cls
    if cls is None:
      cls = self._filter_class
    off = c.hdg_off_for(cls)
    if off == 0.0 and all(abs(float(x)) < 1e-6 for x in c.hdg_off):
      return None
    return wrap_deg(c.push_hdg_ref + off)

  def _home_plan(self):
    """回库几何相对场心：左下发车先朝左(+90)再朝发车(+180)。
    不用推箱偏角（偏角可整体平移标定，与回库方向无关）。"""
    c = self._cfg
    href = float(c.push_hdg_ref)
    layout = int(c.start_layout)
    if layout == 2:
      return wrap_deg(href + 90.0), wrap_deg(href + 180.0)
    if layout == 3:
      return wrap_deg(href - 90.0), wrap_deg(href + 180.0)
    if layout == 4:
      return wrap_deg(href + 90.0), None
    return wrap_deg(href + 180.0), None

  def _enter_home(self):
    y1, y2 = self._home_plan()
    self._yaw_target = y1
    self._home_y2 = y2
    self._home_deadline = ticks_add(ticks_ms(), int(self._cfg.home_timeout_ms))
    self._boundary_armed = False
    self._boundary_pending = False
    self._home_turn_ok = 0
    self._take_motors()
    self.phase = "HOME"
    if self._tcs.on_line:
      self._sub = "LEAVE_LINE"
    else:
      self._sub = "LEG1_TURN"
    self._phase_ms = ticks_ms()
    info("MATCH", "→ HOME sub=%s y1=%.1f y2=%s" % (
      self._sub, y1, ("%.1f" % y2) if y2 is not None else "-"))

  def _tick_home(self, sensors):
    now = ticks_ms()
    if ticks_diff(now, self._home_deadline) > 0:
      self._fault("HOME timeout — gate not confirmed")
      return
    on_line = self._on_line(sensors)
    sub = self._sub
    if sub == "LEAVE_LINE":
      self._home_leave_line(now, on_line)
    elif sub == "LEG1_TURN":
      self._home_leg1_turn(now)
    elif sub == "LEG1_DRIVE":
      self._home_leg1_drive(now, on_line)
    elif sub == "BACKOFF":
      self._home_backoff(now, on_line)
    elif sub == "BACKOFF_TURN":
      self._home_backoff_turn(now)
    elif sub == "LEG2_DRIVE":
      self._home_leg2_drive(on_line)

  def _home_leave_line(self, now, on_line):
    if not on_line:
      self._sub = "LEG1_TURN"
      self._phase_ms = now
      info("MATCH", "HOME → LEG1_TURN")
      return
    self._write_move(-float(self._cfg.drive_duty), 0.0)

  def _home_leg1_turn(self, now):
    if self._spin_toward(self._yaw_target):
      self._hold_yaw = self._yaw_target
      self._hdg_pid.reset()
      self._home_turn_ok = 0
      self._sub = "LEG1_DRIVE"
      self._phase_ms = now
      self._tcs.reset_crossed()
      info("MATCH", "HOME → LEG1_DRIVE hold=%.1f" % self._hold_yaw)

  def _home_leg1_drive(self, now, on_line):
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

  def _home_backoff(self, now, on_line):
    # 至少后退一段时间，避免刚压线抖动就进 TURN 漏掉回头
    min_ms = int(self._cfg.backoff_retreat_min_ms)
    if ticks_diff(now, self._phase_ms) < min_ms:
      self._write_move(-float(self._cfg.drive_duty), 0.0)
      return
    if not on_line or ticks_diff(now, self._phase_ms) > int(self._cfg.home_backoff_ms):
      self._hold_brake()
      self._home_turn_ok = 0
      self._sub = "BACKOFF_TURN"
      self._phase_ms = now
      info("MATCH", "HOME → BACKOFF turn → %.1f" % float(self._home_y2))
      return
    self._write_move(-float(self._cfg.drive_duty), 0.0)

  def _home_backoff_turn(self, now):
    if self._home_y2 is None:
      self._finish()
      return
    if self._spin_toward(self._home_y2):
      self._hold_yaw = self._home_y2
      self._yaw_target = self._home_y2
      self._hdg_pid.reset()
      self._home_turn_ok = 0
      self._sub = "LEG2_DRIVE"
      self._phase_ms = now
      self._tcs.reset_crossed()
      info("MATCH", "HOME → LEG2_DRIVE hold=%.1f" % self._hold_yaw)

  def _home_leg2_drive(self, on_line):
    if on_line:
      self._finish()
      return
    if abs(self._yaw_err(self._hold_yaw)) > 12.0:
      self._spin_toward(self._hold_yaw)
      return
    self._write_move_locked(float(self._cfg.drive_duty), self._hold_yaw)

  def _finish(self):
    self._brake()
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
    elif self.phase == "HOME":
      if self._sub in ("LEG1_DRIVE", "BACKOFF", "LEG2_DRIVE"):
        target_yaw = self._hold_yaw
      elif self._sub == "BACKOFF_TURN" and self._home_y2 is not None:
        target_yaw = self._home_y2
    elif self.phase == "BACKOFF" and self._backoff_sub == "SPIN":
      target_yaw = self._yaw_target
    yaw = self._yaw()
    return (
      self.phase, self._sub, yaw, target_yaw, wrap_deg(target_yaw - yaw),
      cx, y2, self._cmd_forward, self._cmd_lateral, self._cmd_rotation,
      self._orbit_confirm, self._vision_lost)

  @property
  def status_text(self):
    if self._sub:
      return "Match:%s/%s scored=%d" % (self.phase, self._sub, self.scored_count)
    return "Match:%s scored=%d" % (self.phase, self.scored_count)
