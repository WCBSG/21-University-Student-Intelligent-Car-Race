from time import ticks_ms, ticks_diff, ticks_add
from log import info
from motion import MotionControl, wrap_deg
_SPIN_CIRCLE_DEG = 360.0
class MatchHunt:
  def _enter_hunt(self, reverse=False, tracking=False, forward=False):
    self._see_streak = 0
    kept = self._active_cls
    self._set_pick_class()
    if kept is not None and tracking:
      self._lock_active_class(kept)
    self._take_motors()
    self._arm_boundary_when_clear()
    if not self._imu.is_calibrated:
      self._fault("HUNT failed (IMU not ready?)")
      return
    self._confirm_n = 0
    self._lost_n = 0
    self._queue_miss = 0
    self._queue_see = 0
    self._queue_see_cls = -1
    self._align_sweep_active = False
    if reverse:
      self._hunt_search_dir = -self._hunt_search_dir
    if self._hunt_search_dir == 0:
      self._hunt_search_dir = 1
    self._spin_acc = 0.0
    self._rev_start_yaw = self._yaw()
    self._search_target_yaw = self._rev_start_yaw
    self._bearing_pid.reset()
    self._track_ms = ticks_ms()
    self.phase = "HUNT"
    self._phase_ms = ticks_ms()
    if tracking:
      self._sub = "TRACK"
    elif forward:
      self._sub = "FWD"
    else:
      self._sub = "SPIN"
    info("MATCH", "→ HUNT sub=%s cls=%s reverse=%s" % (
      self._sub, self._active_cls, reverse))
  def _arm_boundary_when_clear(self):
    self._boundary_armed = False
    self._yellow_hit = False
    self._tcs.reset_crossed()
    if not self._field_entered:
      self._boundary_pending = True
      self._boundary_need_cross = True
      self._boundary_saw_line = bool(self._tcs.on_line)
      self._boundary_arm_ms = ticks_ms()
      info("MATCH", "boundary wait enter (cross start line)")
      return
    self._boundary_need_cross = False
    self._boundary_saw_line = False
    if self._tcs.on_line:
      self._boundary_pending = True
      info("MATCH", "boundary pending (on yellow)")
    else:
      self._boundary_pending = False
      self._boundary_armed = True
      info("MATCH", "boundary armed")
  def _abort_repick(self, why):
    info("MATCH", "%s" % why)
    self._brake()
    self._enter_hunt()
  def _enter_push(self):
    self._take_motors()
    self._approach_deadline = 0
    self._phase_ms = ticks_ms()
    push_yaw = self._push_yaw()
    self._hold_yaw = self._yaw() if push_yaw is None else float(push_yaw)
    self._hdg_pid.reset()
    self._bearing_pid.reset()
    self._push_lost_n = 0
    self._push_skew_n = 0
    self._push_seen = False
    self._push_last_cx = 50.0
    self._push_correct_prev_cx = 50.0
    self._push_last_y2 = 0.0
    self._push_frame_ms = self._phase_ms
    self._push_cx_rate = 0.0
    self._push_slipped = False
    self._push_heading_bad = 0
    self._evade_ms = 0
    self._evade_clear_n = 0
    self._evade_dir = 0.0
    self._tcs.reset_crossed()
    self.phase = "PUSH"
    self._sub = "DRIVE"
    info("MATCH", "→ PUSH")
  def _push_cx_ok(self, cx):
    return (float(self._cfg.push_cx_left_min) <= float(cx) <=
            float(self._cfg.push_cx_right_max))
  def _push_occlusion_ok(self):
    if self._push_slipped or self._sub == "CORRECT":
      return False
    if not self._push_seen:
      return False
    if not self._push_cx_ok(self._push_last_cx):
      return False
    return self._push_last_y2 >= float(self._cfg.tracking_stage_bottom_pct)
  def _push_reseek(self, why):
    self._abort_repick("PUSH reseek — %s" % why)
  def _enter_align(self, target_yaw):
    self._take_motors()
    self._yaw_target = float(target_yaw)
    self._phase_ms = ticks_ms()
    self._approach_deadline = ticks_add(
      self._phase_ms, int(self._cfg.approach_cluster_timeout_ms))
    self._orbit_confirm = 0
    self._orbit_backoff = True
    self._orbit_backoff_yaw = self._yaw()
    self._vision_lost = 0
    self._was_yaw_ok = False
    self._hdg_pid.reset()
    self._bearing_pid.reset()
    self.phase = "ALIGN"
    self._sub = "TURN"
    self._match_allow = [0, 1, 2]
    info("MATCH", "→ ALIGN push_yaw=%.1f cur=%.1f" % (
      self._yaw_target, self._yaw()))
  def _tick_leave(self, sensors):
    now = ticks_ms()
    target = sensors.get("target") if sensors else None
    if self._seen_target(sensors) and target is not None and int(target[0]) in (0, 1, 2):
      self._lock_active_class(target[0])
      self._arm_boundary_when_clear()
      if not self._imu.is_calibrated:
        self._fault("HUNT failed (IMU not ready?)")
        return
      ty = self._push_yaw()
      y2 = float(target[9])
      stage = float(self._cfg.tracking_stage_bottom_pct)
      if (ty is not None and
          y2 >= stage - 5.0):
        info("MATCH", "LEAVE → see target → ALIGN cls=%s" % self._active_cls)
        self._enter_align(ty)
      else:
        info("MATCH", "LEAVE → see target → HUNT cls=%s" % self._active_cls)
        self._enter_hunt(tracking=True)
      return
    on_line = self._on_line(sensors)
    if self._sub == "EXIT":
      if ticks_diff(now, self._phase_ms) > int(self._cfg.drive_timeout_ms):
        self._enter_leave_shift()
        return
      if on_line:
        self._leave_saw_line = True
      if self._leave_saw_line and not on_line:
        self._enter_leave_shift()
        return
      self._write_move_locked(float(self._cfg.drive_duty), self._hold_yaw)
      return
    if self._sub == "SHIFT":
      if ticks_diff(now, self._phase_ms) > int(self._cfg.leave_shift_ms):
        info("MATCH", "LEAVE SHIFT done → HUNT")
        self._enter_hunt()
        return
      self._write_lateral_locked(
        float(self._cfg.leave_shift_duty),
        self._hold_yaw,
        self._leave_shift_dir)
      return
  def _enter_leave_shift(self):
    self._field_entered = True
    self._boundary_need_cross = False
    self._boundary_saw_line = False
    shift_dir = self._get_leave_shift()
    self._leave_shift_dir = shift_dir
    if shift_dir == 0:
      info("MATCH", "LEAVE crossed line → straight → HUNT")
      self._enter_hunt(forward=True)
    else:
      self._sub = "SHIFT"
      self._phase_ms = ticks_ms()
      label = {1: "RIGHT", -1: "LEFT"}.get(shift_dir, str(shift_dir))
      info("MATCH", "LEAVE crossed line → SHIFT dir=%s" % label)
  def _write_lateral_locked(self, speed, yaw_tgt, shift_dir):
    if self._backoff_busy:
      return
    lat = float(speed) * float(shift_dir)
    self._write_heading_locked(0.0, lat, yaw_tgt)
  def _lock_active_class(self, cls_id):
    self._active_cls = int(cls_id)
    self._filter_class = self._active_cls
    self._match_allow = None
  def _hunt_arrive_y2(self):
    c = self._cfg
    if c.match_mode != "pre":
      return min(float(c.tracking_stage_bottom_pct),
                 float(c.tracking_contact_bottom_pct) - 5.0)
    return float(c.tracking_stop_bottom_pct)
  def _on_hunt_arrived(self, sensors):
    t = sensors.get("target") if sensors else None
    if t is None:
      self._abort_repick("HUNT arrived but no target")
      return
    self._lock_active_class(t[0])
    ty = self._push_yaw()
    if ty is None:
      cx = float(t[6])
      if self._push_cx_ok(cx):
        self._enter_push()
      else:
        self._abort_repick("push skip — cx=%.1f not ahead" % cx)
    else:
      self._enter_align(ty)
  def _hunt_begin_reverse(self):
    self._sub = "SPIN"
    self._hunt_search_dir = -self._hunt_search_dir
    if self._hunt_search_dir == 0:
      self._hunt_search_dir = 1
    self._spin_acc = 0.0
    self._rev_start_yaw = self._yaw()
    self._search_target_yaw = self._rev_start_yaw
    self._confirm_n = 0
    self._lost_n = 0
    info("MATCH", "HUNT lost → flip spin dir=%d" % self._hunt_search_dir)
  def _hunt_queue_update(self, sensors):
    if self._active_cls is not None:
      return
    if not self._remaining or len(self._remaining) < 2:
      return
    if not sensors or not sensors.get("new_frame"):
      return
    n = int(getattr(self._cfg, "pick_class_frames", 6))
    if n < 1:
      n = 1
    has = bool(sensors.get("has_target"))
    t = sensors.get("target")
    if has and t is not None:
      cls = int(t[0])
      self._queue_miss = 0
      if cls == self._queue_see_cls:
        self._queue_see += 1
      else:
        self._queue_see_cls = cls
        self._queue_see = 1
      if self._queue_see >= n and self._remaining[0] != cls:
        if cls in self._remaining:
          self._remaining.remove(cls)
        self._remaining.insert(0, cls)
        info("MATCH", "HUNT see cls=%d ×%d → head rem=%s" % (
          cls, n, self._remaining))
        self._set_pick_class()
        self._queue_see = 0
        self._queue_see_cls = -1
      return
    self._queue_see = 0
    self._queue_see_cls = -1
    self._queue_miss += 1
    if self._queue_miss >= n:
      head = self._remaining.pop(0)
      self._remaining.append(head)
      info("MATCH", "HUNT miss ×%d head cls=%s → end, new=%s" % (
        n, head, self._remaining[0]))
      self._set_pick_class()
      self._queue_miss = 0
  def _tick_hunt_fwd(self, sensors, now):
    if sensors and sensors.get("brick_blocking"):
      self._enter_hunt_evade(sensors, now, "FWD")
      return
    has_tgt = bool(sensors and sensors.get("has_target"))
    if sensors and sensors.get("new_frame"):
      if has_tgt:
        self._confirm_n += 1
      else:
        self._confirm_n = 0
      if self._confirm_n >= int(self._cfg.tracking_confirm_frames):
        t = sensors.get("target")
        if t is not None:
          self._lock_active_class(t[0])
        self._sub = "TRACK"
        self._lost_n = 0
        self._bearing_pid.reset()
        self._track_ms = now
        info("MATCH", "HUNT FWD → TRACK")
        return
    fwd_ms = int(getattr(self._cfg, "center_fwd_ms", 4000))
    if ticks_diff(now, self._phase_ms) > fwd_ms:
      self._sub = "SPIN"
      self._spin_acc = 0.0
      self._rev_start_yaw = self._yaw()
      self._confirm_n = 0
      info("MATCH", "HUNT FWD timeout → SPIN")
      return
    self._write_move_locked(float(self._cfg.drive_duty), self._hold_yaw)
  def _tick_hunt_spin(self, sensors, now):
    has_tgt = bool(sensors and sensors.get("has_target"))
    if sensors and sensors.get("new_frame"):
      if has_tgt:
        self._confirm_n += 1
      else:
        self._confirm_n = 0
      if self._confirm_n >= int(self._cfg.tracking_confirm_frames):
        t = sensors.get("target")
        if t is not None:
          self._lock_active_class(t[0])
        self._sub = "TRACK"
        self._lost_n = 0
        self._bearing_pid.reset()
        self._track_ms = now
        info("MATCH", "HUNT SPIN → TRACK")
        return
      suspect = sensors.get("suspect_target")
      if (suspect is not None and
          ticks_diff(now, self._observe_cooldown_ms) >= 0):
        self._sub = "OBSERVE"
        self._observe_ms = now
        self._hold_brake()
        info("MATCH", "HUNT suspect cls=%d sc=%d → OBSERVE" % (
          int(suspect[0]), int(suspect[1])))
        return
    if has_tgt:
      self._coast()
      return
    yaw = self._yaw()
    d = wrap_deg(yaw - self._rev_start_yaw)
    self._spin_acc += abs(d)
    self._rev_start_yaw = yaw
    if self._spin_acc >= _SPIN_CIRCLE_DEG:
      self._hunt_search_dir = -self._hunt_search_dir
      if self._hunt_search_dir == 0:
        self._hunt_search_dir = 1
      self._spin_acc = 0.0
      self._search_target_yaw = yaw
      info("MATCH", "HUNT SPIN flip dir=%d" % self._hunt_search_dir)
    self._write_spin_rate(
      float(self._cfg.tracking_search_speed) * self._hunt_search_dir)
  def _tick_hunt_observe(self, sensors, now):
    self._hold_brake()
    if sensors and sensors.get("new_frame") and sensors.get("has_target"):
      self._confirm_n += 1
      if self._confirm_n >= int(self._cfg.tracking_confirm_frames):
        t = sensors.get("target")
        if t is not None:
          self._lock_active_class(t[0])
        self._sub = "TRACK"
        self._lost_n = 0
        self._bearing_pid.reset()
        self._track_ms = now
        info("MATCH", "HUNT OBSERVE → TRACK")
        return
    elif sensors and sensors.get("new_frame"):
      self._confirm_n = 0
    if ticks_diff(now, self._observe_ms) >= int(self._cfg.tracking_observe_ms):
      self._sub = "SPIN"
      self._confirm_n = 0
      self._rev_start_yaw = self._yaw()
      self._search_target_yaw = self._rev_start_yaw
      self._observe_cooldown_ms = ticks_add(
        now, int(self._cfg.tracking_observe_cooldown_ms))
      info("MATCH", "HUNT OBSERVE timeout → SPIN")
  def _enter_hunt_evade(self, sensors, now, return_sub="TRACK"):
    brick = sensors.get("brick") if sensors else None
    self._hunt_evade_dir = (
      -1.0 if brick is not None and float(brick[6]) >= 50.0 else 1.0)
    self._hunt_evade_yaw = self._yaw()
    self._hunt_evade_ms = now
    self._hunt_evade_return = return_sub
    self._sub = "EVADE"
    info("MATCH", "HUNT → EVADE dir=%+.0f" % self._hunt_evade_dir)
  def _tick_hunt_evade(self, now):
    if ticks_diff(now, self._hunt_evade_ms) >= int(self._cfg.hunt_evade_ms):
      self._sub = self._hunt_evade_return
      self._lost_n = 0
      self._track_ms = now
      self._bearing_pid.reset()
      info("MATCH", "HUNT EVADE → %s" % self._hunt_evade_return)
      return
    self._write_heading_locked(
      float(self._cfg.hunt_evade_forward_duty),
      self._hunt_evade_dir * float(self._cfg.hunt_evade_lateral_duty),
      self._hunt_evade_yaw)
  def _tick_hunt_track(self, sensors, now):
    has_tgt = bool(sensors and sensors.get("has_target"))
    y2 = float(sensors.get("y2", 0.0)) if sensors else 0.0
    if sensors and sensors.get("brick_blocking"):
      self._enter_hunt_evade(sensors, now, "TRACK")
      return
    if sensors and sensors.get("new_frame"):
      if has_tgt and y2 >= self._hunt_arrive_y2():
        self._hold_brake()
        self._on_hunt_arrived(sensors)
        return
      if has_tgt:
        self._lost_n = 0
      else:
        self._lost_n += 1
      if self._lost_n >= int(self._cfg.tracking_lost_frames):
        self._hunt_begin_reverse()
        return
    t = sensors.get("target") if sensors else None
    if t is None:
      self._coast()
      return
    be = (float(t[6]) - 50.0) / 50.0
    real_dt = ticks_diff(now, self._track_ms) / 1000.0
    if real_dt <= 0.0 or real_dt > 0.5:
      real_dt = 0.1
    self._track_ms = now
    be_dt = ticks_diff(now, self._be_ms) / 1000.0
    be_rate = (be - self._prev_be) / be_dt if 0.001 < be_dt < 0.5 else 0.0
    self._prev_be = be
    self._be_ms = now
    rot = (float(self._cfg.tracking_bearing_actuation_sign) *
           self._bearing_pid.update(be, real_dt, be_rate))
    self._write_vector(
      float(self._cfg.tracking_approach_speed), 0.0, rot, False)
  def _tick_hunt(self, sensors):
    if sensors and sensors.get("cam_timeout"):
      self._fault("cam timeout in HUNT")
      return
    now = ticks_ms()
    if ticks_diff(now, self._phase_ms) > int(self._cfg.pick_timeout_ms):
      self._skip_or_home("HUNT timeout")
      return
    if self._sub in ("SPIN", "FWD"):
      self._hunt_queue_update(sensors)
    if self._sub == "TRACK":
      self._tick_hunt_track(sensors, now)
    elif self._sub == "OBSERVE":
      self._tick_hunt_observe(sensors, now)
    elif self._sub == "EVADE":
      self._tick_hunt_evade(now)
    elif self._sub == "FWD":
      self._tick_hunt_fwd(sensors, now)
    else:
      self._tick_hunt_spin(sensors, now)
  def _align_lost_soft(self, sensors):
    self._coast()
    if sensors and sensors.get("new_frame"):
      self._vision_lost += 1
    wait = int(self._cfg.orbit_lost_frames)
    if self._vision_lost < wait:
      return False
    search_spd = float(self._cfg.tracking_search_speed)
    if not self._align_sweep_active:
      self._align_sweep_active = True
      self._align_search_dir = -self._align_search_dir
      if self._align_search_dir == 0:
        self._align_search_dir = 1
      self._search_target_yaw = self._yaw()
      info("MATCH", "ALIGN lost → PD sweep dir=%d" % self._align_search_dir)
    if self._vision_lost < wait * 4:
      self._write_spin_rate(search_spd * self._align_search_dir)
      return False
    info("MATCH", "ALIGN lost long → HUNT")
    self._enter_hunt(reverse=True)
    return True
  def _write_orbit_pd(self, forward, lateral, rotation):
    fwd = float(forward) * MotionControl._FWD_K
    side = -float(lateral) * MotionControl._SIDE_K
    duties = self._mix_duties
    duties[0] = self._clamp(fwd + side + rotation, -100.0, 100.0)
    duties[1] = self._clamp(-fwd + side + rotation, -100.0, 100.0)
    duties[2] = self._clamp(-2.0 * side + rotation, -100.0, 100.0)
    self._set_command(forward, lateral, rotation)
    self._arb.write(self.OWNER, duties, False)
  def _tick_align(self, sensors):
    c = self._cfg
    now = ticks_ms()
    if (self._approach_deadline and
        ticks_diff(now, self._approach_deadline) > 0):
      self._skip_or_home("ALIGN total timeout")
      return
    if ticks_diff(now, self._phase_ms) > int(c.orbit_timeout_ms):
      info("MATCH", "ALIGN timeout → HUNT")
      self._enter_hunt(reverse=True)
      return
    target = sensors.get("target") if sensors else None
    if target is None:
      self._align_lost_soft(sensors)
      return
    if sensors and sensors.get("new_frame"):
      self._vision_lost = 0
      self._align_sweep_active = False
      tgt_cls = int(target[0])
      if tgt_cls != self._active_cls:
        self._active_cls = tgt_cls
        self._filter_class = tgt_cls
    cx = float(target[6])
    y2 = float(target[9])
    yaw_err = self._yaw_err(self._yaw_target)
    yaw_rate = self._yaw_rate()
    yaw_tol = float(c.orbit_yaw_tol_deg)
    cx_tol = float(c.orbit_center_tol_pct)
    lat_spd = float(c.orbit_speed)
    contact = float(c.tracking_contact_bottom_pct)
    cx_off = cx - 50.0
    yaw_ok = abs(yaw_err) <= yaw_tol
    if not yaw_ok and self._was_yaw_ok:
      yaw_ok = abs(yaw_err) <= yaw_tol * 1.5
    self._was_yaw_ok = yaw_ok
    near_contact = y2 >= float(c.tracking_contact_bottom_pct)
    cx_ok = abs(cx_off) <= (cx_tol * 2.0 if near_contact else cx_tol)
    stage_y2 = float(c.tracking_stage_bottom_pct)
    tgt_cls = int(target[0])
    if tgt_cls == 1:
      stage_y2 = max(stage_y2 - 15.0, 60.0)
    radial = (stage_y2 - y2) * float(c.orbit_radial_kp)
    radial = self._clamp(radial, -float(c.orbit_radial_max), float(c.orbit_radial_max))
    translate_mode = abs(yaw_err) <= float(c.orbit_translate_yaw_deg)
    if yaw_ok or translate_mode:
      lateral = self._clamp(
        (cx - 50.0) / 50.0 * lat_spd, -lat_spd, lat_spd)
      dt = self._control_dt()
      rot = (float(c.yaw_actuation_sign) *
             self._hdg_pid.update(yaw_err, dt, yaw_rate))
      cx_abs = abs(cx - 50.0)
      if yaw_ok:
        approach = float(c.tracking_final_approach_speed)
        approach *= max(0.5, 1.0 - cx_abs / 50.0)
      else:
        approach = self._clamp(radial, -8.0, 8.0)
      self._write_vector(approach, lateral, rot, use_min_duty=True)
      ready = yaw_ok and cx_ok and y2 >= contact
      if sensors and sensors.get("new_frame"):
        self._orbit_confirm = self._orbit_confirm + 1 if ready else 0
      if self._orbit_confirm >= int(c.orbit_confirm_frames):
        self._hold_brake()
        info("MATCH", "ALIGN → PUSH yaw=%.1f cx=%.1f" % (self._yaw(), cx))
        self._enter_push()
      return
    if self._orbit_backoff:
      min_done = ticks_diff(now, self._phase_ms) >= int(c.orbit_backoff_min_ms)
      if (not min_done) or y2 > stage_y2:
        self._write_move_locked(
          -float(c.orbit_backoff_duty), self._orbit_backoff_yaw)
        return
      self._orbit_backoff = False
    edge = abs(cx_off) / 50.0
    if edge > 1.0:
      edge = 1.0
    edge_scale = 1.0 - 0.4 * edge
    abs_err = abs(yaw_err)
    brake_start = float(c.orbit_brake_start_deg)
    translate_deg = float(c.orbit_translate_yaw_deg)
    if abs_err >= brake_start:
      strength = 1.0
    else:
      span_deg = max(1.0, brake_start - translate_deg)
      ratio = self._clamp(
        (abs_err - translate_deg) / span_deg, 0.0, 1.0)
      min_scale = float(c.orbit_brake_min_scale)
      strength = min_scale + (1.0 - min_scale) * ratio
    dt = self._control_dt()
    pid_out = self._hdg_pid.update(yaw_err, dt, yaw_rate)
    dir_s = float(c.yaw_actuation_sign)
    if pid_out == 0.0:
      rot_dir = 1 if (dir_s * yaw_err) >= 0.0 else -1
    else:
      rot_dir = 1 if (dir_s * pid_out) >= 0.0 else -1
    rot_n = rot_dir * strength * edge_scale
    rot_n = self._clamp(rot_n, -1.0, 1.0)
    spin = rot_n * float(c.orbit_front_spin)
    min_spin = float(c.orbit_min_spin)
    if abs(spin) < min_spin:
      spin = min_spin * (1 if spin >= 0 else -1)
    span = max(1.0, contact - stage_y2)
    clearance = self._clamp((contact - y2) / span, 0.45, 1.0)
    slip = rot_n * float(c.orbit_front_slip) * clearance
    if bool(c.orbit_front_flip):
      slip = -slip
    min_slip = float(c.orbit_min_slip) * max(0.7, clearance)
    if abs(slip) < min_slip:
      slip = min_slip * (1 if slip >= 0 else -1)
    lat_extra = self._clamp(
      (cx - 50.0) / 50.0 * lat_spd * 0.4, -lat_spd * 0.4, lat_spd * 0.4)
    side_total = slip + lat_extra
    fwd_duty = radial if radial < 0 else max(0.0, radial * 0.5)
    self._write_orbit_pd(fwd_duty, side_total, spin)
    if sensors and sensors.get("new_frame"):
      self._orbit_confirm = 0
  def _push_watch_frame(self, sensors, elapsed):
    if not sensors or not sensors.get("new_frame"):
      return None
    need = int(self._cfg.push_watch_frames)
    t = sensors.get("target")
    if t is None:
      if sensors.get("brick_blocking"):
        self._push_lost_n = 0
        return None
      if elapsed < int(self._cfg.push_entry_grace_ms):
        return "grace"
      if (elapsed >= int(self._cfg.push_lost_blind_ms) and
          self._push_occlusion_ok()):
        self._push_lost_n = 0
        self._push_skew_n = 0
        return "ok"
      self._push_skew_n = 0
      self._push_lost_n += 1
      if self._push_lost_n >= need:
        return "reseek"
      return None
    cx = float(t[6])
    y2 = float(t[9])
    now = ticks_ms()
    frame_dt = ticks_diff(now, self._push_frame_ms) / 1000.0
    if 0.02 <= frame_dt <= 0.5:
      self._push_cx_rate = (cx - self._push_correct_prev_cx) / frame_dt
    else:
      self._push_cx_rate = 0.0
    self._push_correct_prev_cx = cx
    self._push_frame_ms = now
    self._push_seen = True
    self._push_last_cx = cx
    self._push_last_y2 = y2
    if self._push_cx_ok(cx):
      self._push_lost_n = 0
      self._push_skew_n = 0
      return "ok"
    self._push_slipped = True
    self._push_lost_n = 0
    self._push_skew_n += 1
    if self._push_skew_n >= need:
      return "correct"
    return None
  def _write_push_correct(self, sensors):
    t = sensors.get("target") if sensors else None
    if t is None:
      self._hold_brake()
      return
    cx = float(t[6])
    err_cx = cx - 50.0
    lateral = self._clamp(
      err_cx * float(self._cfg.push_correct_lateral_kp) -
      self._push_cx_rate * float(self._cfg.push_correct_lateral_kd),
      -float(self._cfg.push_correct_lateral_max),
      float(self._cfg.push_correct_lateral_max))
    self._write_heading_locked(
      float(self._cfg.push_correct_duty), lateral, self._hold_yaw)
  def _tick_push(self, sensors):
    now = ticks_ms()
    elapsed = ticks_diff(now, self._phase_ms)
    if elapsed > int(self._cfg.push_timeout_ms):
      info("MATCH", "PUSH timeout %dms — NOT scored" % elapsed)
      self._brake()
      self._skip_or_home("PUSH timeout")
      return
    if abs(self._yaw_err(self._hold_yaw)) > float(
        self._cfg.push_heading_realign_deg):
      self._push_heading_bad += 1
      if self._push_heading_bad >= int(self._cfg.push_heading_realign_frames):
        self._hold_brake()
        info("MATCH", "PUSH heading lost → ALIGN")
        self._enter_align(self._hold_yaw)
        return
    else:
      self._push_heading_bad = 0
    watch = self._push_watch_frame(sensors, elapsed)
    if watch == "reseek":
      self._push_reseek("lost %d frames" % self._push_lost_n)
      return
    blocking = bool(sensors and sensors.get("brick_blocking"))
    if blocking:
      if self._sub != "EVADE":
        brick = sensors.get("brick")
        self._evade_dir = (
          -1.0 if brick is not None and float(brick[6]) >= 50.0 else 1.0)
        self._evade_ms = now
        self._sub = "EVADE"
        info("MATCH", "PUSH → EVADE dir=%+.0f" % self._evade_dir)
      self._evade_clear_n = 0
    if self._sub == "EVADE":
      if sensors and sensors.get("new_frame") and not blocking:
        self._evade_clear_n += 1
      elif blocking:
        self._evade_clear_n = 0
      elapsed_evade = ticks_diff(now, self._evade_ms)
      if (elapsed_evade >= int(self._cfg.push_evade_min_ms) and
          self._evade_clear_n >= int(self._cfg.push_evade_clear_frames)):
        self._sub = "DRIVE"
        self._hdg_pid.reset()
        info("MATCH", "PUSH EVADE complete → DRIVE")
      else:
        self._write_heading_locked(
          float(self._cfg.push_evade_forward_duty),
          self._evade_dir * float(self._cfg.push_evade_lateral_duty),
          self._hold_yaw)
        return
    if watch == "correct":
      if self._sub != "CORRECT":
        self._sub = "CORRECT"
        self._push_correct_prev_cx = self._push_last_cx
        self._bearing_pid.reset()
        info("MATCH", "PUSH → CORRECT (skew)")
    elif watch == "ok" and self._sub == "CORRECT":
      self._sub = "DRIVE"
      self._hdg_pid.reset()
      info("MATCH", "PUSH CORRECT → DRIVE")
    if self._sub == "CORRECT":
      self._write_push_correct(sensors)
      return
    push_duty = float(self._cfg.push_duty)
    if self._push_last_y2 >= float(self._cfg.push_slow_zone_y2):
      push_duty = min(push_duty, float(self._cfg.push_slow_duty))
    self._write_move_locked(push_duty, self._hold_yaw)
