# match_hunt.py — 场内找物/推送 phase mixin（单独编译）
# HUNT(搜+跟) / ALIGN(绕轴对齐+贴近) / PUSH：Match 全程占用电机
from time import ticks_ms, ticks_diff, ticks_add
from log import info
from motion import MotionControl, wrap_deg

_SPIN_CIRCLE_DEG = 360.0


class MatchHunt:
  def _enter_hunt(self, reverse=False, tracking=False, forward=False):
    self._see_streak = 0
    kept = self._active_cls
    self._set_pick_class()
    # 已锁定类别时保留（_set_pick_class 会清成 None）
    if kept is not None and tracking:
      self._lock_active_class(kept)
    self._cache_backoff_duties()
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
    self._search_phase = "reverse" if reverse else "spin"
    if reverse:
      self._search_dir = -self._search_dir
    if self._search_dir == 0:
      self._search_dir = 1
    self._rev_acc = 0.0
    self._spin_acc = 0.0
    self._rev_start_yaw = self._yaw()
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
    """武装场锁。出库前：须压过起点黄线再离线；进场后：离线即武装。"""
    self._boundary_armed = False
    self._yellow_hit = False
    self._tcs.reset_crossed()
    if not self._field_entered:
      # 库外/出库中：禁止立刻武装，否则压进场黄线会误触发 BACKOFF
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
    self._hold_yaw = self._yaw()
    self._hdg_pid.reset()
    self._bearing_pid.reset()
    self._ctrl_ms = self._phase_ms
    self._push_bad = 0
    self._push_bad_kind = ""
    self._push_seen = False
    self._push_last_cx = 50.0
    self._push_last_y2 = 0.0
    self._push_slipped = False
    self._tcs.reset_crossed()
    self.phase = "PUSH"
    self._sub = "DRIVE"
    info("MATCH", "→ PUSH")

  def _push_cx_ok(self, cx):
    return (float(self._cfg.push_cx_left_min) <= float(cx) <=
            float(self._cfg.push_cx_right_max))

  def _push_occlusion_ok(self):
    """贴头遮挡才继续推：末帧仍居中且近；划走/过偏后丢失则否。"""
    if self._push_slipped or self._sub == "CORRECT":
      return False
    if not self._push_seen:
      return False
    if not self._push_cx_ok(self._push_last_cx):
      return False
    return self._push_last_y2 >= float(self._cfg.tracking.stage_bottom_pct)

  def _push_reseek(self, why):
    self._abort_repick("PUSH reseek — %s" % why)

  def _enter_align(self, target_yaw):
    self._take_motors()
    self._yaw_target = float(target_yaw)
    self._phase_ms = ticks_ms()
    self._approach_deadline = ticks_add(
      self._phase_ms, int(self._cfg.approach_cluster_timeout_ms))
    self._ctrl_ms = self._phase_ms
    self._orbit_confirm = 0
    self._vision_lost = 0
    self._hdg_pid.reset()
    self._bearing_pid.reset()
    self._hdg_ms = self._phase_ms
    self.phase = "ALIGN"
    self._sub = "TURN"
    info("MATCH", "→ ALIGN push_yaw=%.1f cur=%.1f" % (
      self._yaw_target, self._yaw()))

  def _tick_leave(self, sensors):
    now = ticks_ms()
    target = sensors.get("target") if sensors else None
    if self._seen_target(sensors) and target is not None:
      self._lock_active_class(target[0])
      self._cache_backoff_duties()
      self._arm_boundary_when_clear()
      if not self._imu.is_calibrated:
        self._fault("HUNT failed (IMU not ready?)")
        return
      # 已近且需对位 → 直进 ALIGN；否则 HUNT 跟踪
      ty = self._push_yaw()
      y2 = float(target[9])
      stage = float(self._cfg.tracking.stage_bottom_pct)
      if (ty is not None and
          y2 >= stage - 5.0):
        info("MATCH", "LEAVE → see target → ALIGN cls=%s" % self._active_cls)
        self._enter_align(ty)
      else:
        info("MATCH", "LEAVE → see target → HUNT cls=%s" % self._active_cls)
        self._enter_hunt(tracking=True)
      return
    if ticks_diff(now, self._phase_ms) > int(self._cfg.drive_timeout_ms):
      info("MATCH", "LEAVE timeout → HUNT")
      self._enter_hunt()
      return
    self._write_move_locked(float(self._cfg.drive_duty), self._hold_yaw)

  def _lock_active_class(self, cls_id):
    self._active_cls = int(cls_id)
    self._filter_class = self._active_cls
    self._match_allow = None

  def _hunt_arrive_y2(self):
    """决赛：stage 或已很近都可进 ALIGN；预赛：stop/contact。"""
    tr = self._cfg.tracking
    if getattr(self._cfg, "match_mode", "final") != "pre":
      stage = float(tr.stage_bottom_pct)
      contact = float(tr.contact_bottom_pct)
      return min(stage, contact - 5.0)
    return float(tr.stop_bottom_pct)

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
    self._search_phase = "spin"
    self._search_dir = -self._search_dir
    if self._search_dir == 0:
      self._search_dir = 1
    self._spin_acc = 0.0
    self._rev_start_yaw = self._yaw()
    self._confirm_n = 0
    self._lost_n = 0
    info("MATCH", "HUNT lost → flip spin dir=%d" % self._search_dir)

  def _hunt_queue_update(self, sensors):
    """队首连续 n 帧无物→队尾；连续 n 帧见某物→置队首。仅搜索态。"""
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
    """回中区：朝 hold_yaw 前进寻物；见目标→TRACK；超时→SPIN。"""
    has_tgt = bool(sensors and sensors.get("has_target"))
    if sensors and sensors.get("new_frame"):
      if has_tgt:
        self._confirm_n += 1
      else:
        self._confirm_n = 0
      if self._confirm_n >= int(self._cfg.tracking.confirm_frames):
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
      self._phase_ms = now  # SPIN 重新起算搜索超时
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
      if self._confirm_n >= int(self._cfg.tracking.confirm_frames):
        t = sensors.get("target")
        if t is not None:
          self._lock_active_class(t[0])
        self._sub = "TRACK"
        self._search_phase = "spin"
        self._lost_n = 0
        self._bearing_pid.reset()
        self._track_ms = now
        info("MATCH", "HUNT SPIN → TRACK")
        return
    if has_tgt:
      self._hold_brake()
      return
    # 正一圈 / 反一圈
    yaw = self._yaw()
    d = wrap_deg(yaw - self._rev_start_yaw)
    self._spin_acc += abs(d)
    self._rev_start_yaw = yaw
    if self._spin_acc >= _SPIN_CIRCLE_DEG:
      self._search_dir = -self._search_dir
      if self._search_dir == 0:
        self._search_dir = 1
      self._spin_acc = 0.0
      info("MATCH", "HUNT SPIN flip dir=%d" % self._search_dir)
    s = float(self._cfg.tracking.search_speed) * self._search_dir
    self._write_spin(s)

  def _tick_hunt_track(self, sensors, now):
    has_tgt = bool(sensors and sensors.get("has_target"))
    y2 = float(sensors.get("y2", 0.0)) if sensors else 0.0
    if sensors and sensors.get("new_frame"):
      if has_tgt and y2 >= self._hunt_arrive_y2():
        self._hold_brake()
        self._on_hunt_arrived(sensors)
        return
      if has_tgt:
        self._lost_n = 0
      else:
        self._lost_n += 1
      if self._lost_n >= int(self._cfg.tracking.lost_frames):
        self._hunt_begin_reverse()
        return
    t = sensors.get("target") if sensors else None
    if t is None:
      self._hold_brake()
      return
    be = (float(t[6]) - 50.0) / 50.0
    real_dt = ticks_diff(now, self._track_ms) / 1000.0
    if real_dt <= 0.0 or real_dt > 0.5:
      real_dt = 0.1
    self._track_ms = now
    rot = (float(self._cfg.tracking.bearing_actuation_sign) *
           self._bearing_pid.update(be, real_dt))
    fwd = MotionControl.move(float(self._cfg.tracking.approach_speed), 0.0)
    duties = [
      self._clamp(fwd[i] + rot, -100.0, 100.0) for i in range(3)
    ]
    self._set_command(float(self._cfg.tracking.approach_speed), 0.0, rot)
    self._arb.write(self.OWNER, duties)

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
    elif self._sub == "FWD":
      self._tick_hunt_fwd(sensors, now)
    else:
      self._tick_hunt_spin(sensors, now)

  def _align_lost_soft(self, sensors):
    """丢目标：刹停等待；久丢则小反转找球；不立刻 skip。返回 True=已回 HUNT。"""
    self._hold_brake()
    if sensors and sensors.get("new_frame"):
      self._vision_lost += 1
    # 短丢：只等
    wait = int(self._cfg.orbit_lost_frames)
    if self._vision_lost < wait:
      return False
    # 中丢：原地反转找
    if self._vision_lost == wait:
      self._search_dir = -self._search_dir
      if self._search_dir == 0:
        self._search_dir = 1
      info("MATCH", "ALIGN lost → reverse find")
    if self._vision_lost < wait * 4:
      s = float(self._cfg.tracking.search_speed) * self._search_dir
      self._write_spin(s)
      return False
    # 久丢：回 HUNT（不换类 skip）
    info("MATCH", "ALIGN lost long → HUNT")
    self._enter_hunt(reverse=True)
    return True

  def _tick_align(self, sensors):
    """TURN：航向不对→绕行；航向已对→全向平移居中；CLOSE：前进贴接触 → PUSH。"""
    now = ticks_ms()
    if (self._approach_deadline and
        ticks_diff(now, self._approach_deadline) > 0):
      self._skip_or_home("ALIGN total timeout")
      return
    if ticks_diff(now, self._phase_ms) > int(self._cfg.orbit_timeout_ms):
      info("MATCH", "ALIGN timeout → HUNT")
      self._enter_hunt(reverse=True)
      return
    target = sensors.get("target") if sensors else None
    if target is None:
      self._align_lost_soft(sensors)
      return
    if sensors and sensors.get("new_frame"):
      self._vision_lost = 0

    cx = float(target[6])
    y2 = float(target[9])
    yaw_err = self._yaw_err(self._yaw_target)
    yaw_tol = float(self._cfg.orbit_yaw_tol_deg)
    cx_tol = float(self._cfg.orbit_center_tol_pct)
    lat_spd = float(self._cfg.orbit_speed)
    contact = float(self._cfg.tracking.contact_bottom_pct)
    cx_off = cx - 50.0
    yaw_ok = abs(yaw_err) <= yaw_tol
    cx_ok = abs(cx_off) <= cx_tol
    stage_y2 = float(self._cfg.tracking.stage_bottom_pct)
    radial = (stage_y2 - y2) * float(self._cfg.orbit_radial_kp)
    radial = self._clamp(
      radial,
      -float(self._cfg.orbit_radial_max),
      float(self._cfg.orbit_radial_max))

    # CLOSE：航向+居中已齐，前进贴接触；偏了优先侧移纠，不回绕行
    if self._sub == "CLOSE":
      if abs(yaw_err) > yaw_tol * 1.5:
        self._sub = "TURN"
        self._orbit_confirm = 0
        self._hdg_pid.reset()
        info("MATCH", "ALIGN CLOSE → TURN (yaw)")
        return
      if y2 >= contact:
        if self._push_cx_ok(cx):
          self._hold_brake()
          info("MATCH", "ALIGN → PUSH")
          self._enter_push()
        else:
          # 接触距离但过偏：全向侧移回中，不绕行
          lateral = self._clamp(
            (50.0 - cx) / 50.0 * lat_spd, -lat_spd, lat_spd)
          dt = self._control_dt()
          rot = (float(self._cfg.yaw_actuation_sign) *
                 self._hdg_pid.update(yaw_err, dt))
          self._write_vector(0.0, lateral, rot)
        return
      # 前进贴近：cx 偏则侧移优先，少用自旋
      lateral = self._clamp(
        (50.0 - cx) / 50.0 * lat_spd, -lat_spd, lat_spd)
      dt = self._control_dt()
      rot = (float(self._cfg.yaw_actuation_sign) *
             self._hdg_pid.update(yaw_err, dt) * 0.35)
      self._write_vector(
        float(self._cfg.tracking.final_approach_speed), lateral, rot)
      return

    # TURN：推方向已对 → 全向平移把目标移到镜头中心；否则才绕前方轴
    if yaw_ok:
      lateral = self._clamp(
        (50.0 - cx) / 50.0 * lat_spd, -lat_spd, lat_spd)
      dt = self._control_dt()
      # 轻锁航向，避免侧移时漂角
      rot = (float(self._cfg.yaw_actuation_sign) *
             self._hdg_pid.update(yaw_err, dt) * 0.4)
      self._write_vector(radial, lateral, rot)
      ready = cx_ok
      if sensors and sensors.get("new_frame"):
        self._orbit_confirm = self._orbit_confirm + 1 if ready else 0
      if self._orbit_confirm >= int(self._cfg.orbit_confirm_frames):
        self._sub = "CLOSE"
        self._bearing_pid.reset()
        self._hold_yaw = self._yaw_target
        self._hdg_pid.reset()
        info("MATCH", "ALIGN strafe → CLOSE cx=%.1f" % cx)
      return

    # 航向不对：绕前方轴转到 push_yaw
    edge = abs(cx_off) / 50.0
    if edge > 1.0:
      edge = 1.0
    spin_scale = 1.0 - 0.7 * edge
    dt = self._control_dt()
    rot_n = (float(self._cfg.yaw_actuation_sign) *
             self._hdg_pid.update(yaw_err, dt) * spin_scale)
    rot_n = self._clamp(rot_n / 40.0, -1.0, 1.0)
    spin = rot_n * float(self._cfg.orbit_front_spin)
    slip = rot_n * float(self._cfg.orbit_front_slip)
    if bool(self._cfg.orbit_front_flip):
      slip = -slip
    lat_extra = self._clamp(
      (50.0 - cx) / 50.0 * lat_spd * 0.4, -lat_spd * 0.4, lat_spd * 0.4)
    self._write_orbit_front(spin, slip + lat_extra, radial)
    if sensors and sensors.get("new_frame"):
      self._orbit_confirm = 0  # 航向未齐，不算居中确认

  def _push_watch_frame(self, sensors, elapsed):
    if not sensors or not sensors.get("new_frame"):
      return None
    need = int(self._cfg.push_watch_frames)
    t = sensors.get("target")
    if t is None:
      # 旧逻辑：盲区后一律当遮挡继续推 → 物体划走仍全速冲黄线
      if (elapsed >= int(self._cfg.push_lost_blind_ms) and
          self._push_occlusion_ok()):
        self._push_bad = 0
        self._push_bad_kind = ""
        return "ok"
      if self._push_bad_kind != "lost":
        self._push_bad = 0
      self._push_bad_kind = "lost"
      self._push_bad += 1
      if self._push_bad >= need:
        return "reseek"
      return None
    cx = float(t[6])
    y2 = float(t[9])
    self._push_seen = True
    self._push_last_cx = cx
    self._push_last_y2 = y2
    if self._push_cx_ok(cx):
      self._push_bad = 0
      self._push_bad_kind = ""
      return "ok"
    self._push_slipped = True
    if self._push_bad_kind != "skew":
      self._push_bad = 0
    self._push_bad_kind = "skew"
    self._push_bad += 1
    if self._push_bad >= need:
      return "correct"
    return None

  def _write_push_correct(self, sensors):
    t = sensors.get("target") if sensors else None
    if t is None:
      self._hold_brake()
      return
    bearing = (float(t[6]) - 50.0) / 50.0
    rot = (float(self._cfg.tracking.bearing_actuation_sign) *
           self._bearing_pid.update(bearing, self._control_dt()))
    self._write_vector(float(self._cfg.push_correct_duty), 0.0, rot)

  def _tick_push(self, sensors):
    now = ticks_ms()
    elapsed = ticks_diff(now, self._phase_ms)
    if elapsed > int(self._cfg.push_timeout_ms):
      info("MATCH", "PUSH timeout %dms — NOT scored" % elapsed)
      self._brake()
      self._skip_or_home("PUSH timeout")
      return
    watch = self._push_watch_frame(sensors, elapsed)
    if watch == "reseek":
      self._push_reseek("lost %d frames" % self._push_bad)
      return
    if watch == "correct":
      if self._sub != "CORRECT":
        self._sub = "CORRECT"
        self._bearing_pid.reset()
        info("MATCH", "PUSH → CORRECT (skew)")
    elif watch == "ok" and self._sub == "CORRECT":
      self._sub = "DRIVE"
      self._hdg_pid.reset()
      info("MATCH", "PUSH CORRECT → DRIVE")
    if self._sub == "CORRECT":
      self._write_push_correct(sensors)
      return
    self._write_move_locked(float(self._cfg.push_duty), self._hold_yaw)
