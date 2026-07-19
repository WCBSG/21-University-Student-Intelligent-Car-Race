# match.py — MatchRunner 核心 + HOME（ISR/Hunt 为 mixin，分文件编译）
# 主车：视觉/运动/计分/回库。单一 Match 状态机，全程占用电机。
from time import ticks_ms, ticks_diff, ticks_add
from log import info, flush as log_flush
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
    self._filter_class = int(cfg.tracking_target_class)
    self._yaw_target = 0.0
    self._hold_yaw = 0.0
    self._home_y2 = None
    self._home_deadline = 0
    self._hdg_pid = HeadingPID(kp=cfg.heading_kp, kd=cfg.heading_kd, max_output=cfg.heading_max, deadband=cfg.heading_db)
    self._bearing_pid = HeadingPID(kp=cfg.tracking_kp, kd=cfg.tracking_kd, max_output=cfg.tracking_max, deadband=cfg.tracking_db)
    self._track_ms = ticks_ms()
    self._sample_yaw = self._imu.get_yaw()
    self._sample_rate = 0.0
    self._sample_dt = 0.02
    self._sample_ms = ticks_ms()
    self._heading_rot_cmd = 0.0
    self._spin_rate_cmd = 0.0
    self._prev_be = 0.0
    self._be_ms = ticks_ms()
    self._search_target_yaw = 0.0
    self._observe_ms = 0
    self._observe_cooldown_ms = 0
    self._hunt_evade_ms = 0
    self._hunt_evade_dir = 0.0
    self._hunt_evade_yaw = 0.0
    self._hunt_evade_return = "TRACK"
    self._orbit_confirm = 0
    self._orbit_backoff = True
    self._orbit_backoff_yaw = 0.0
    self._vision_lost = 0
    self._align_sweep_active = False
    self._confirm_n = 0
    self._lost_n = 0
    self._hunt_search_dir = 1
    self._align_search_dir = 1
    self._rev_start_yaw = 0.0
    self._approach_deadline = 0
    self._cmd_forward = 0.0
    self._cmd_lateral = 0.0
    self._cmd_rotation = 0.0
    self.fault_reason = ""
    self._push_lost_n = 0
    self._push_skew_n = 0
    self._push_seen = False
    self._push_last_cx = 50.0
    self._push_correct_prev_cx = 50.0
    self._push_last_y2 = 0.0
    self._push_frame_ms = 0
    self._push_cx_rate = 0.0
    self._push_slipped = False
    self._push_heading_bad = 0
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
    self._bo_spin = [0.0, 0.0, 0.0]
    self._mix_duties = [0.0, 0.0, 0.0]
    self._spin_duties = [0.0, 0.0, 0.0]
    self._spin_good = 0
    self._spin_start_yaw = 0.0
    self._backoff_spin_dir = 1
    self._backoff_retreat_yaw = 0.0
    self._home_turn_ok = 0
    self._home_turn_dir = 0
    self._home_turn_ms = 0
    self._home_xb_wait_ms = 0
    self._queue_miss = 0
    self._queue_see = 0
    self._queue_see_cls = -1
    self._spin_acc = 0.0
    self._observe_ms = 0
    self._observe_cooldown_ms = 0
    self._hunt_evade_ms = 0
    self._hunt_evade_dir = 0.0
    self._hunt_evade_yaw = 0.0
    self._hunt_evade_return = "TRACK"
    self._align_sweep_active = False
    self._leave_saw_line = False
    self._leave_shift_dir = 0
    self._was_yaw_ok = False
    self._push_heading_bad = 0
    self._backoff_spin_dir = 1
    self._home_turn_dir = 0
    self._home_turn_ms = 0
    self._home_xb_wait_ms = 0

  def _get_leave_shift(self):
    """出库平移方向: +1=右移, -1=左移, 0=直行"""
    layout = int(self._cfg.start_layout)
    if layout in (0, 1):      # 底边中 → 直行
      return 0
    if layout == 2:           # 左下角 → 右移
      return 1
    if layout == 3:           # 右下角 → 左移
      return -1
    return 0

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
    self._hunt_search_dir = 1
    self._align_search_dir = 1
    self._sub = ""
    self._phase_ms = ticks_ms()
    self._filter_class = 7
    self._match_allow = None
    self._hold_yaw = self._yaw()
    self._hdg_pid.reset()
    self._sample_yaw = self._imu.get_yaw()
    self._sample_rate = 0.0
    self._sample_dt = 0.02
    self._sample_ms = ticks_ms()
    self._heading_rot_cmd = 0.0
    self._spin_rate_cmd = 0.0
    self._prev_be = 0.0
    self._be_ms = ticks_ms()
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
    self._leave_saw_line = False
    self._leave_shift_dir = 0
    self._was_yaw_ok = False
    self.phase = "LEAVE"
    self._sub = "EXIT"
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
    self._filter_class = int(self._cfg.tracking_target_class)
    self._brake()

  def tick(self, dt, sensors):
    self._sample_control(dt)
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
    return self._sample_yaw

  def _yaw_err(self, target):
    return wrap_deg(target - self._yaw())

  def _yaw_rate(self):
    return self._sample_rate

  def _sample_control(self, dt):
    """每个主循环只采样一次 yaw/dt/rate，供全部 PD 路径复用。"""
    cur = self._imu.get_yaw()
    now = ticks_ms()
    elapsed_ms = ticks_diff(now, self._sample_ms)
    self._sample_ms = now
    dt = elapsed_ms / 1000.0
    valid = 0.001 < dt <= 0.5
    if valid:
      raw_rate = wrap_deg(cur - self._sample_yaw) / dt
      tau = max(0.001, float(self._cfg.yaw_rate_lpf_tau))
      alpha = dt / (tau + dt)
      rate = self._sample_rate + alpha * (raw_rate - self._sample_rate)
    else:
      dt = 0.02
      rate = 0.0
    self._sample_yaw = cur
    self._sample_rate = rate
    self._sample_dt = dt

  def _take_motors(self):
    self._arb.acquire(self.OWNER)

  def _on_line(self, sensors):
    if sensors and sensors.get("tcs_on_line"):
      return True
    return bool(self._tcs.on_line)

  def _set_command(self, forward, lateral, rotation):
    self._cmd_forward = float(forward)
    self._cmd_lateral = float(lateral)
    self._cmd_rotation = float(rotation)

  def _write_move_locked(self, speed, yaw_tgt):
    self._write_heading_locked(speed, 0.0, yaw_tgt)

  def _write_heading_locked(self, forward, lateral, yaw_tgt,
                            use_min_duty=False, allow_backoff=False):
    """IMU 航向 PD 封装：平移为前馈，旋转分量闭环锁定 yaw。"""
    if self._backoff_busy:
      if not allow_backoff:
        return
    rot = self._heading_rotation(yaw_tgt)
    self._write_vector(
      forward, lateral, rot, use_min_duty, allow_backoff)

  def _heading_rotation(self, yaw_tgt):
    """返回保持指定航向所需的旋转分量。"""
    err = self._yaw_err(yaw_tgt)
    target = self._cfg.yaw_actuation_sign * self._hdg_pid.update(
      err, self._sample_dt, self._sample_rate)
    self._heading_rot_cmd = self._slew_rotation(
      self._heading_rot_cmd, target,
      float(self._cfg.heading_slew_duty_s))
    return self._heading_rot_cmd

  def _write_spin_rate(self, desired_rate):
    """角速度闭环搜索，避免累计目标航向跨过 ±180°后输出反转。"""
    err_rate = float(desired_rate) - self._sample_rate
    target = (float(self._cfg.yaw_actuation_sign) *
              float(self._cfg.tracking_spin_rate_kp) * err_rate)
    limit = float(self._cfg.tracking_spin_max_duty)
    target = self._clamp(target, -limit, limit)
    self._spin_rate_cmd = self._slew_rotation(
      self._spin_rate_cmd, target,
      float(self._cfg.tracking_spin_slew_duty_s))
    self._write_spin(self._spin_rate_cmd)

  def _slew_rotation(self, current, target, duty_per_s):
    """限制旋转输出变化率，正反切换时必须平滑经过零点。"""
    step = max(0.1, duty_per_s * min(self._sample_dt, 0.05))
    delta = target - current
    if delta > step:
      return current + step
    if delta < -step:
      return current - step
    return target

  def _control_dt(self):
    return self._sample_dt

  def _write_vector(self, forward, lateral, rot, use_min_duty=False,
                    allow_backoff=False):
    """全向运动：forward=前后, lateral=左右(>0右移), rot=自旋(>0 CW)。"""
    if self._backoff_busy and not allow_backoff:
      return
    f = float(forward) * MotionControl._FWD_K
    s = float(lateral) * MotionControl._SIDE_K
    duties = self._mix_duties
    duties[0] = self._clamp(f + s + rot, -100.0, 100.0)
    duties[1] = self._clamp(-f + s + rot, -100.0, 100.0)
    duties[2] = self._clamp(-2.0 * s + rot, -100.0, 100.0)
    self._set_command(forward, lateral, rot)
    self._arb.write(self.OWNER, duties, use_min_duty)

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
    duties = self._spin_duties
    duties[0] = duties[1] = duties[2] = d
    self._arb.write(self.OWNER, duties, True)

  def _hold_brake(self):
    self._set_command(0.0, 0.0, 0.0)
    self._arb.hold_brake(self.OWNER)

  def _coast(self):
    """短暂视觉丢帧时滑行，避免电子刹车造成机械点头和震颤。"""
    self._set_command(0.0, 0.0, 0.0)
    duties = self._spin_duties
    duties[0] = duties[1] = duties[2] = 0.0
    self._arb.write(self.OWNER, duties, False)

  def _brake(self):
    self._set_command(0.0, 0.0, 0.0)
    self._heading_rot_cmd = 0.0
    self._spin_rate_cmd = 0.0
    self._arb.force_brake()

  def _start_home_turn(self, target):
    self._yaw_target = float(target)
    err = self._yaw_err(self._yaw_target)
    self._home_turn_dir = 1 if err >= 0.0 else -1
    self._home_turn_ms = ticks_ms()
    self._home_turn_ok = 0
    self._hdg_pid.reset()

  def _spin_toward(self, target):
    err = self._yaw_err(target)
    rate = self._yaw_rate()
    tol = float(self._cfg.align_tol_deg)
    if abs(err) <= tol and abs(rate) < float(self._cfg.home_turn_rate_tol):
      self._home_turn_ok += 1
      if self._home_turn_ok >= int(self._cfg.home_turn_confirm_frames):
        self._hold_brake()
        self._home_turn_ok = 0
        self._home_turn_dir = 0
        return True
      self._hold_brake()
      return False
    if (self._home_turn_ms and
        ticks_diff(ticks_ms(), self._home_turn_ms) >
        int(self._cfg.home_turn_timeout_ms)):
      self._fault("HOME turn timeout target=%.1f yaw=%.1f" % (
        target, self._yaw()))
      return False
    self._home_turn_ok = 0
    control_err = err
    if abs(err) > float(self._cfg.turn_latch_release_deg):
      # 180°附近 wrap 符号会因噪声翻转；保持进入转向时的方向。
      if self._home_turn_dir == 0:
        self._home_turn_dir = 1 if err >= 0.0 else -1
      control_err = abs(err) * self._home_turn_dir
    else:
      self._home_turn_dir = 0
    raw = (float(self._cfg.home_turn_kp) * control_err -
           float(self._cfg.home_turn_kd) * rate)
    limit = float(self._cfg.home_turn_max_duty)
    s = float(self._cfg.yaw_actuation_sign) * self._clamp(
      raw, -limit, limit)
    self._write_spin(s)
    return False

  def _fault(self, why):
    info("MATCH", "FAULT: %s" % why)
    log_flush()
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

  # 硬编码推箱方向 (IMU yaw): cls=0沙包→90°, cls=1网球→0°, cls=2熊→-90°
  _PUSH_HDG = {0: 90.0, 1: 0.0, 2: -90.0}

  def _push_yaw(self):
    """决赛推箱车头朝向。预赛返回 None。"""
    c = self._cfg
    if c.match_mode == "pre":
      return None
    cls = self._active_cls
    if cls is None:
      cls = self._filter_class
    return self._PUSH_HDG.get(int(cls))

  def _home_plan(self):
    """回库 yaw: layout=1直180°; 2=先90°再180°; 3=先-90°再180°"""
    layout = int(self._cfg.start_layout)
    if layout == 2:
      return 90.0, 180.0
    if layout == 3:
      return -90.0, 180.0
    return 180.0, None  # layout 1: 底边中

  def _enter_home(self):
    y1, y2 = self._home_plan()
    self._hold_yaw = self._yaw()
    self._yaw_target = y1
    self._home_y2 = y2
    self._home_deadline = ticks_add(ticks_ms(), int(self._cfg.home_timeout_ms))
    self._boundary_armed = False
    self._boundary_pending = False
    self._home_turn_ok = 0
    self._take_motors()
    self.phase = "HOME"
    self._sub = "SETTLE"
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
    if sub == "SETTLE":
      self._home_settle(now, on_line)
    elif sub == "LEAVE_LINE":
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
      self._home_leg2_drive(now, on_line, sensors)
    elif sub == "LEG2_WAIT_XB":
      self._home_wait_xb(now, sensors)
    elif sub == "CROSS":
      self._home_cross(now)

  def _home_settle(self, now, on_line):
    self._hold_brake()
    elapsed = ticks_diff(now, self._phase_ms)
    if elapsed < int(self._cfg.home_mag_settle_ms):
      return
    if (not self._imu.mag_ready and
        elapsed < int(self._cfg.home_mag_settle_max_ms)):
      return
    self._hold_yaw = self._yaw()
    mag_rel = self._imu.get_mag_rel()
    info("MATCH", "HOME settle yaw=%.1f mag=%s off=%.1f rel=%s still=%d" % (
      self._hold_yaw,
      "ready" if self._imu.mag_ready else "timeout",
      self._imu.fused_offset,
      ("%.1f" % mag_rel) if mag_rel is not None else "-",
      self._imu.still_count))
    self._phase_ms = now
    if on_line:
      self._sub = "LEAVE_LINE"
    else:
      self._sub = "LEG1_TURN"
      self._start_home_turn(self._yaw_target)

  def _home_leave_line(self, now, on_line):
    if not on_line:
      self._sub = "LEG1_TURN"
      self._phase_ms = now
      self._start_home_turn(self._yaw_target)
      info("MATCH", "HOME → LEG1_TURN")
      return
    self._write_move_locked(-float(self._cfg.drive_duty), self._hold_yaw)

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
        self._hold_brake()
        self._sub = "CROSS"
        self._phase_ms = now
        info("MATCH", "HOME → CROSS")
        return
      self._hold_brake()
      self._sub = "BACKOFF"
      self._phase_ms = now
      info("MATCH", "HOME → BACKOFF")
      return
    self._write_move_locked(float(self._cfg.drive_duty), self._hold_yaw)

  def _home_backoff(self, now, on_line):
    # 至少后退一段时间，避免刚压线抖动就进 TURN 漏掉回头
    min_ms = int(self._cfg.backoff_retreat_min_ms)
    if ticks_diff(now, self._phase_ms) < min_ms:
      self._write_move_locked(-float(self._cfg.drive_duty), self._hold_yaw)
      return
    if not on_line or ticks_diff(now, self._phase_ms) > int(self._cfg.home_backoff_ms):
      self._hold_brake()
      self._home_turn_ok = 0
      self._sub = "BACKOFF_TURN"
      self._phase_ms = now
      self._start_home_turn(self._home_y2)
      info("MATCH", "HOME → BACKOFF turn → %.1f" % float(self._home_y2))
      return
    self._write_move_locked(-float(self._cfg.drive_duty), self._hold_yaw)

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
      self._match_allow = [int(self._cfg.CLS_XB)]  # LEG2 只追踪 XB 信标
      self._push_last_cx = 50.0
      self._tcs.reset_crossed()
      info("MATCH", "HOME → LEG2_DRIVE hold=%.1f (XB tracking)" % self._hold_yaw)

  def _home_leg2_drive(self, now, on_line, sensors):
    """LEG2 陀螺仪航向锁 + XB 信标横向修正"""
    target = sensors.get("target")
    has_xb = target is not None

    leg2_ready = ticks_diff(now, self._phase_ms) >= int(
      self._cfg.home_leg2_min_ms)
    if on_line and leg2_ready:
      # XB 确认：压黄线 + 信标在视野内且够近
      xb_ok = has_xb and target[9] >= float(self._cfg.home_xb_contact_y2)
      self._hold_brake()
      if xb_ok:
        self._sub = "CROSS"
        self._phase_ms = now
        info("MATCH", "HOME XB confirmed (y2=%.0f) → CROSS" % target[9])
      elif bool(self._cfg.home_require_xb):
        self._sub = "LEG2_WAIT_XB"
        self._home_xb_wait_ms = now
        info("MATCH", "HOME yellow → wait XB")
      else:
        self._sub = "CROSS"
        self._phase_ms = now
        info("MATCH", "HOME yellow (XB miss/too far) → CROSS")
      return

    # 未压线：航向锁 + XB 横向修正
    lateral = 0.0
    if has_xb:
      cx = float(target[6])
      err_cx = cx - 50.0       # XB 偏右→正→右移
      d_cx = cx - self._push_last_cx
      self._push_last_cx = cx
      kp = float(self._cfg.home_xb_lateral_kp)
      kd = float(self._cfg.home_xb_lateral_kd)
      lateral = self._clamp(err_cx * kp - d_cx * kd, -60.0, 60.0)

    self._write_heading_locked(
      float(self._cfg.drive_duty), lateral, self._hold_yaw)

  def _home_wait_xb(self, now, sensors):
    self._hold_brake()
    target = sensors.get("target") if sensors else None
    if (target is not None and
        target[9] >= float(self._cfg.home_xb_contact_y2)):
      self._sub = "CROSS"
      self._phase_ms = now
      info("MATCH", "HOME XB acquired while waiting → CROSS")
      return
    if ticks_diff(now, self._home_xb_wait_ms) > int(self._cfg.home_xb_wait_ms):
      self._fault("HOME yellow reached but XB not confirmed")

  def _home_cross(self, now):
    """过线后继续前进 1 秒，防止停在线上。"""
    if ticks_diff(now, self._phase_ms) >= int(self._cfg.home_cross_ms):
      self._finish()
      return
    self._write_move_locked(float(self._cfg.drive_duty), self._hold_yaw)

  def _finish(self):
    self._brake()
    self.phase = "DONE"
    self._sub = ""
    info("MATCH", "DONE scored=%d" % self.scored_count)
    log_flush()

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
