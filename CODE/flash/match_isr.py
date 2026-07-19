from time import ticks_ms, ticks_diff
from log import info
from motion import wrap_deg
class MatchIsr:
  def flush_deferred(self):
    if self._def_armed:
      self._def_armed = False
      if self._field_entered:
        info("MATCH", "boundary armed (entered)")
      else:
        info("MATCH", "boundary armed (cleared)")
    sc = self._def_score
    if sc:
      self._def_score = 0
      if sc == 1:
        info("MATCH", "SCORE total=%d/%d rem=%s" % (
          self.scored_count, int(self._cfg.match_target_count), self._remaining))
      else:
        info("MATCH", "PUSH yellow but score rejected (false-push guard)")
    bo = self._def_bo
    if bo:
      self._def_bo = 0
      if bo == 1:
        info("MATCH", "yellow → BACKOFF")
      elif bo == 2:
        info("MATCH", "BACKOFF done → HOME")
      elif bo == 3:
        info("MATCH", "BACKOFF done → FWD (center)")
  def check_field_lock(self, on_line=None, now=None):
    if self._backoff_busy or self._yellow_hit:
      return
    if not self.field_lock_enabled:
      return
    if on_line is None:
      on_line = bool(self._tcs.on_line)
    else:
      on_line = bool(on_line)
    if now is None:
      now = ticks_ms()
    if self._boundary_pending:
      if self._boundary_need_cross:
        if on_line:
          self._boundary_saw_line = True
        elif self._boundary_saw_line:
          self._boundary_pending = False
          self._boundary_need_cross = False
          self._boundary_saw_line = False
          self._boundary_armed = True
          self._field_entered = True
          self._def_armed = True
        return
      if not on_line:
        self._boundary_pending = False
        self._boundary_armed = True
        self._def_armed = True
      return
    if self._boundary_armed and on_line:
      self._boundary_armed = False
      self._boundary_pending = False
      self._yellow_hit = True
      self._yellow_hit_phase = self.phase
      self._tcs.reset_crossed()
  def consume_yellow_hit(self):
    if not self._yellow_hit:
      return False
    self._yellow_hit = False
    hit_phase = self._yellow_hit_phase
    self._yellow_hit_phase = ""
    if not self._field_entered:
      info("MATCH", "yellow ignored (not entered yet)")
      return False
    self._arb.force_brake()
    self._arb.acquire(self.OWNER)
    self._set_command(0.0, 0.0, 0.0)
    if hit_phase == "PUSH":
      if self._push_score_ready():
        self._credit_score()
        self._def_score = 1
      else:
        self._def_score = 2
    self._start_backoff()
    return True
  def _start_backoff(self):
    if self._backoff_busy:
      return
    self._def_bo = 1
    self._backoff_retreat_yaw = self._yaw()
    self._yaw_target = wrap_deg(
      self._backoff_retreat_yaw + float(self._cfg.backoff_spin_deg))
    self._backoff_spin_dir = (
      1 if float(self._cfg.backoff_spin_deg) >= 0.0 else -1)
    self._see_streak = 0
    self.phase = "BACKOFF"
    self._sub = "RETREAT"
    self._backoff_sub = "RETREAT"
    self._backoff_ms = ticks_ms()
    self._backoff_busy = True
    self._post_backoff = None
    self._spin_good = 0
    self._write_bo()
  def _backoff_spin_control(self):
    err = self._yaw_err(self._yaw_target)
    control_err = err
    if abs(err) > float(self._cfg.turn_latch_release_deg):
      control_err = abs(err) * self._backoff_spin_dir
    raw = self._cfg.yaw_actuation_sign * self._hdg_pid.update(
      control_err, self._control_dt(), self._yaw_rate())
    limit = float(self._cfg.backoff_spin_max_duty)
    return err, self._clamp(raw, -limit, limit)
  def step_backoff(self):
    if not self._backoff_busy:
      return
    now = ticks_ms()
    if self._backoff_sub == "RETREAT":
      elapsed = ticks_diff(now, self._backoff_ms)
      timed = elapsed > int(self._cfg.recover_backoff_ms)
      if elapsed >= int(self._cfg.backoff_retreat_min_ms):
        on_line = bool(self._tcs.on_line)
        if (not on_line) or timed:
          self._hdg_pid.reset()
          _, s = self._backoff_spin_control()
          self._bo_spin[0] = s
          self._bo_spin[1] = s
          self._bo_spin[2] = s
          self._spin_start_yaw = self._yaw()
          self._backoff_sub = "SPIN"
          self._sub = "SPIN"
          self._backoff_ms = now
          return
      self._write_bo()
      return
    if self._backoff_sub == "SPIN":
      err, s = self._backoff_spin_control()
      self._bo_spin[0] = s
      self._bo_spin[1] = s
      self._bo_spin[2] = s
      self._write_bo(self._bo_spin)
      if abs(err) < float(self._cfg.backoff_spin_tol_deg):
        self._spin_good += 1
        if self._spin_good >= int(self._cfg.backoff_spin_confirm_frames):
          self._finish_backoff(True)
          return
      else:
        self._spin_good = 0
      if ticks_diff(now, self._backoff_ms) > int(self._cfg.backoff_spin_timeout_ms):
        self._finish_backoff(False)
        return
  def _finish_backoff(self, aligned):
    self._hold_yaw = self._yaw_target if aligned else self._yaw()
    self._arb.hold_brake(self.OWNER)
    self._set_command(0.0, 0.0, 0.0)
    self._backoff_sub = "DONE"
    self._sub = "DONE"
    if self._want_home or self.scored_count >= int(self._cfg.match_target_count):
      self._want_home = False
      self._post_backoff = "HOME"
      self._def_bo = 2
    else:
      self._post_backoff = "FWD"
      self._def_bo = 3
    self._backoff_busy = False
  def _write_bo(self, duties=None):
    if self._backoff_sub == "RETREAT":
      self._write_heading_locked(
        -float(self._cfg.drive_duty), 0.0,
        self._backoff_retreat_yaw, False, True)
    else:
      self._arb.write(self.OWNER, duties)
      self._set_command(0.0, 0.0, float(duties[0]))
  def _push_score_ready(self):
    if not self._tcs.on_line:
      return False
    elapsed = ticks_diff(ticks_ms(), self._phase_ms)
    if elapsed < 200:
      return False
    if abs(self._yaw_err(self._hold_yaw)) > float(self._cfg.align_tol_deg) * 2.0:
      return False
    return True
  def _credit_score(self):
    self.scored_count += 1
    if self._active_cls is not None:
      cls = int(self._active_cls)
      if cls in self._remaining:
        self._remaining.remove(cls)
      self._remaining.append(cls)
    if self.scored_count >= int(self._cfg.match_target_count):
      self._want_home = True
