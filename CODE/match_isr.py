# match_isr.py — 场锁 ISR（仅急停+标记）+ 主循环 BACKOFF mixin
# ISR 不跑电机步进 / 不计分；BACKOFF 与得分在主循环完成。
from time import ticks_ms, ticks_diff
from log import info
from motion import MotionControl, wrap_deg

# 兼容旧名：match.py 等可 `from match_isr import wrap`
wrap = wrap_deg


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

  def isr_field_lock(self):
    """50Hz ISR：只急停 + 置黄线标志，不写电机步进、不计分。"""
    if self._backoff_busy or self._yellow_hit:
      return
    if not self.field_lock_enabled:
      return
    on_line = bool(self._tcs.on_line)
    if self._boundary_pending:
      if self._boundary_need_cross:
        # 出库进场：必须先见到黄线再离线，才算进场并武装
        if on_line:
          self._boundary_saw_line = True
        elif self._boundary_saw_line:
          self._boundary_pending = False
          self._boundary_need_cross = False
          self._boundary_saw_line = False
          self._boundary_armed = True
          self._field_entered = True
          self._def_armed = True
        elif (self._boundary_arm_ms and
              ticks_diff(ticks_ms(), self._boundary_arm_ms) > 3000):
          # 一直未见黄线：视为已在场内（起点在线外/内侧）
          self._boundary_pending = False
          self._boundary_need_cross = False
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
      self._arb.force_brake()
      self._arb.acquire(self.OWNER)
      self._set_command(0.0, 0.0, 0.0)
      self._boundary_armed = False
      self._boundary_pending = False
      self._yellow_hit = True
      self._yellow_hit_phase = self.phase
      self._tcs.reset_crossed()

  def consume_yellow_hit(self):
    """主循环：消费黄线标志 → 计分（若 PUSH）→ 启动 BACKOFF。"""
    if not self._yellow_hit:
      return False
    self._yellow_hit = False
    hit_phase = self._yellow_hit_phase
    self._yellow_hit_phase = ""
    # 出库完成前忽略（防库外武装后压出库线误 BACKOFF）
    if not self._field_entered:
      info("MATCH", "yellow ignored (not entered yet)")
      return False
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
    self._arb.force_brake()
    self._arb.acquire(self.OWNER)
    self._yaw_target = wrap_deg(self._yaw() + 180.0)
    self._see_streak = 0
    self.phase = "BACKOFF"
    self._sub = "RETREAT"
    self._backoff_sub = "RETREAT"
    self._backoff_ms = ticks_ms()
    self._backoff_busy = True
    self._post_backoff = None
    self._cache_backoff_duties()
    self._write_bo(self._bo_retreat)

  def step_backoff(self):
    """主循环：BACKOFF 原子步进（后退 → 转 180°）。"""
    if not self._backoff_busy:
      return
    now = ticks_ms()
    if self._backoff_sub == "RETREAT":
      elapsed = ticks_diff(now, self._backoff_ms)
      timed = elapsed > int(self._cfg.recover_backoff_ms)
      # 最少后退，防 reset_crossed 后 on_line 抖动误切 SPIN
      if elapsed >= int(self._cfg.backoff_retreat_min_ms):
        on_line = bool(self._tcs.on_line)
        if (not on_line) or timed:
          err = self._yaw_err(self._yaw_target)
          s = float(self._cfg.drive_duty)
          s = self._cfg.yaw_actuation_sign * (s if err > 0 else -s)
          self._bo_spin[0] = s
          self._bo_spin[1] = s
          self._bo_spin[2] = s
          self._spin_start_yaw = self._yaw()
          self._backoff_sub = "SPIN"
          self._sub = "SPIN"
          self._backoff_ms = now
          return
      self._write_bo(self._bo_retreat)
      return
    if self._backoff_sub == "SPIN":
      turned = abs(wrap_deg(self._yaw() - self._spin_start_yaw))
      if turned >= float(self._cfg.backoff_spin_deg):
        self._finish_backoff(True)
        return
      if ticks_diff(now, self._backoff_ms) > 2000:
        self._finish_backoff(False)
        return
      self._write_bo(self._bo_spin)

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

  def _write_bo(self, duties):
    self._arb.write(self.OWNER, duties)
    if self._backoff_sub == "RETREAT":
      self._set_command(-float(self._cfg.drive_duty), 0.0, 0.0)
    else:
      self._set_command(0.0, 0.0, float(duties[0]))

  def _cache_backoff_duties(self):
    d = -float(self._cfg.drive_duty)
    mv = MotionControl.move(d, 0.0)
    self._bo_retreat[0] = mv[0]
    self._bo_retreat[1] = mv[1]
    self._bo_retreat[2] = mv[2]

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
