# match_hunt.py — 场内找物/推送 phase mixin（单独编译）
# SEARCH/TRACK/FACE/PUSH：Match 全程占用电机（无独立 Robot FSM）
# 决赛对位 FACE：绕车头前方轴转到 push_yaw，同时保球居中、径向保距 → FINAL/PUSH
from time import ticks_ms, ticks_diff, ticks_add
from log import info
from motion import MotionControl, wrap_deg


class MatchHunt:
  def _enter_search(self, reverse=False):
    self._see_streak = 0
    self._set_pick_class()
    self._cache_backoff_duties()
    self._take_motors()
    self._arm_boundary_when_clear()
    if not self._imu.is_calibrated:
      self._fault("SEARCH failed (IMU not ready?)")
      return
    self._confirm_n = 0
    self._lost_n = 0
    self._search_phase = "reverse" if reverse else "spin"
    self._search_dir = -self._search_dir if reverse else self._search_dir
    if self._search_dir == 0:
      self._search_dir = 1
    self._rev_acc = 0.0
    self._rev_start_yaw = self._yaw()
    self._bearing_pid.reset()
    self._track_ms = ticks_ms()
    self.phase = "SEARCH"
    self._phase_ms = ticks_ms()
    self._sub = ""
    info("MATCH", "→ SEARCH cls=%s armed=%s reverse=%s" % (
      self._active_cls, self._boundary_armed, reverse))

  def _enter_track(self):
    self._take_motors()
    self._confirm_n = 0
    self._lost_n = 0
    self._search_phase = "spin"
    self._bearing_pid.reset()
    self._track_ms = ticks_ms()
    self.phase = "TRACK"
    self._phase_ms = ticks_ms()
    self._sub = ""
    info("MATCH", "→ TRACK cls=%s" % self._active_cls)

  def _arm_boundary_when_clear(self):
    self._boundary_armed = False
    self._tcs.reset_crossed()
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
    self._enter_search()

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
    self._tcs.reset_crossed()
    self.phase = "PUSH"
    self._sub = "DRIVE"
    info("MATCH", "→ PUSH")

  def _push_cx_ok(self, cx):
    return (float(self._cfg.push_cx_left_min) <= float(cx) <=
            float(self._cfg.push_cx_right_max))

  def _push_reseek(self, why):
    self._abort_repick("PUSH reseek — %s" % why)

  def _enter_face(self, target_yaw):
    previous_phase = self.phase
    self._take_motors()
    self._yaw_target = float(target_yaw)
    self._phase_ms = ticks_ms()
    if previous_phase != "FINAL_APPROACH":
      self._approach_deadline = (
        ticks_add(self._phase_ms, int(self._cfg.approach_cluster_timeout_ms)))
    self._ctrl_ms = self._phase_ms
    self._orbit_confirm = 0
    self._vision_lost = 0
    self._hdg_pid.reset()
    self._bearing_pid.reset()
    self._hdg_ms = self._phase_ms
    self.phase = "FACE"
    self._sub = ""
    info("MATCH", "→ FACE push_yaw=%.1f cur=%.1f" % (
      self._yaw_target, self._yaw()))

  def _enter_final_approach(self):
    self._phase_ms = ticks_ms()
    self._ctrl_ms = self._phase_ms
    self._vision_lost = 0
    self._bearing_pid.reset()
    self._hold_yaw = self._yaw_target
    self._hdg_pid.reset()
    self.phase = "FINAL_APPROACH"
    self._sub = ""
    info("MATCH", "FACE → FINAL_APPROACH")

  def _vision_lost_tick(self, sensors, why):
    """目标丢失：刹车并累计丢帧；达阈值则 skip。返回 True=已 skip。"""
    self._hold_brake()
    if sensors and sensors.get("new_frame"):
      self._vision_lost += 1
    if self._vision_lost >= int(self._cfg.orbit_lost_frames):
      self._skip_or_home(why)
      return True
    return False

  def _tick_leave(self, sensors):
    now = ticks_ms()
    target = sensors.get("target") if sensors else None
    # 见目标：直接进 TRACK，不继续盲直行
    if self._seen_target(sensors) and target is not None:
      self._lock_active_class(target[0])
      self._cache_backoff_duties()
      self._arm_boundary_when_clear()
      if not self._imu.is_calibrated:
        self._fault("TRACK failed (IMU not ready?)")
        return
      info("MATCH", "LEAVE → see target → TRACK cls=%s" % self._active_cls)
      self._enter_track()
      return
    # 未见目标：锁航向直行，超时再 SEARCH
    if ticks_diff(now, self._phase_ms) > int(self._cfg.drive_timeout_ms):
      info("MATCH", "LEAVE timeout → SEARCH")
      self._enter_search()
      return
    self._write_move_locked(float(self._cfg.drive_duty), self._hold_yaw)

  def _lock_active_class(self, cls_id):
    self._active_cls = int(cls_id)
    self._filter_class = self._active_cls
    self._match_allow = None

  def _tick_search(self, sensors):
    if sensors and sensors.get("cam_timeout"):
      self._fault("cam timeout in SEARCH")
      return
    now = ticks_ms()
    if ticks_diff(now, self._phase_ms) > int(self._cfg.pick_timeout_ms):
      self._skip_or_home("SEARCH timeout")
      return
    if (self._remaining and len(self._remaining) > 1 and
        ticks_diff(now, self._phase_ms) > int(self._cfg.pick_class_timeout_ms)):
      head = self._remaining.pop(0)
      self._remaining.append(head)
      info("MATCH", "SEARCH head cls=%s → end, new head=%s" % (
        head, self._remaining[0]))
      self._set_pick_class()
      self._phase_ms = now
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
        info("MATCH", "SEARCH → TRACK")
        self._enter_track()
        return
    # 驱动：见目标且非反转 → 刹停等确认；否则自旋/反转弧
    if has_tgt and self._search_phase != "reverse":
      self._hold_brake()
      return
    if self._search_phase == "reverse":
      yaw = self._yaw()
      d = wrap_deg(yaw - self._rev_start_yaw)
      self._rev_acc += abs(d)
      self._rev_start_yaw = yaw
      if self._rev_acc >= self._cfg.tracking.reverse_angle:
        self._search_phase = "spin"
        self._rev_acc = 0.0
    s = float(self._cfg.tracking.search_speed) * self._search_dir
    self._write_spin(s)

  def _track_stop_pct(self):
    # 决赛停在 staging；预赛直接接触位
    if getattr(self._cfg, "match_mode", "final") != "pre":
      return float(self._cfg.tracking.stage_bottom_pct)
    return float(self._cfg.tracking.stop_bottom_pct)

  def _on_track_arrived(self, sensors):
    t = sensors.get("target") if sensors else None
    if t is None:
      self._abort_repick("TRACK arrived but no target")
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
      self._enter_face(ty)

  def _tick_track(self, sensors):
    if sensors and sensors.get("cam_timeout"):
      self._fault("cam timeout in TRACK")
      return
    now = ticks_ms()
    if ticks_diff(now, self._phase_ms) > int(self._cfg.approach_timeout_ms):
      self._skip_or_home("TRACK timeout")
      return
    has_tgt = bool(sensors and sensors.get("has_target"))
    y2 = float(sensors.get("y2", 0.0)) if sensors else 0.0
    if sensors and sensors.get("new_frame"):
      if has_tgt and y2 >= self._track_stop_pct():
        self._hold_brake()
        self._on_track_arrived(sensors)
        return
      if has_tgt:
        self._lost_n = 0
      else:
        self._lost_n += 1
      if self._lost_n >= int(self._cfg.tracking.lost_frames):
        info("MATCH", "TRACK lost → SEARCH reverse")
        self._enter_search(reverse=True)
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

  def _tick_face(self, sensors):
    """绕前方轴转到 push_yaw：M1/M2 慢、M3 快；并保球居中/径向距离。"""
    now = ticks_ms()
    if (self._approach_deadline and
        ticks_diff(now, self._approach_deadline) > 0):
      self._skip_or_home("FACE/FINAL total timeout")
      return
    if ticks_diff(now, self._phase_ms) > int(self._cfg.orbit_timeout_ms):
      self._skip_or_home("FACE timeout")
      return
    target = sensors.get("target") if sensors else None
    if target is None:
      self._vision_lost_tick(sensors, "FACE target lost")
      return
    if sensors and sensors.get("new_frame"):
      self._vision_lost = 0
    cx = float(target[6])
    y2 = float(target[9])
    yaw_err = self._yaw_err(self._yaw_target)
    stage_y2 = float(self._cfg.tracking.stage_bottom_pct)
    yaw_tol = float(self._cfg.orbit_yaw_tol_deg)
    cx_tol = float(self._cfg.orbit_center_tol_pct)
    lat_spd = float(self._cfg.orbit_speed)
    cx_off = cx - 50.0
    # 球越偏，转向越慢，优先侧移把球拉回中心
    edge = abs(cx_off) / 50.0
    if edge > 1.0:
      edge = 1.0
    spin_scale = 1.0 - 0.7 * edge
    dt = self._control_dt()
    # PID 归一到 [-1,1]，再乘满额绕轴占空比
    rot_n = (float(self._cfg.yaw_actuation_sign) *
             self._hdg_pid.update(yaw_err, dt) * spin_scale)
    rot_n = self._clamp(rot_n / 40.0, -1.0, 1.0)
    spin_max = float(self._cfg.orbit_front_spin)
    slip_max = float(self._cfg.orbit_front_slip)
    spin = rot_n * spin_max
    slip = rot_n * slip_max
    if bool(self._cfg.orbit_front_flip):
      slip = -slip
    radial = (stage_y2 - y2) * float(self._cfg.orbit_radial_kp)
    radial = self._clamp(
      radial,
      -float(self._cfg.orbit_radial_max),
      float(self._cfg.orbit_radial_max))
    # 航向已齐：只保距+居中；未齐：绕前方轴转（可叠少量 cx 侧移）
    if abs(yaw_err) <= yaw_tol:
      lateral = self._clamp(
        (50.0 - cx) / 50.0 * lat_spd, -lat_spd, lat_spd)
      self._write_vector(radial, lateral, 0.0)
    else:
      # 球偏时额外侧移，避免绕轴时球出画
      lat_extra = self._clamp(
        (50.0 - cx) / 50.0 * lat_spd * 0.4, -lat_spd * 0.4, lat_spd * 0.4)
      self._write_orbit_front(spin, slip + lat_extra, radial)
    # 距离不卡 FACE：FINAL/PUSH 会继续贴近；只要求航向+居中
    aligned = (
      abs(yaw_err) <= yaw_tol and
      abs(cx_off) <= cx_tol)
    if sensors and sensors.get("new_frame"):
      self._orbit_confirm = self._orbit_confirm + 1 if aligned else 0
    if self._orbit_confirm >= int(self._cfg.orbit_confirm_frames):
      self._hold_brake()
      self._enter_final_approach()

  def _tick_final_approach(self, sensors):
    now = ticks_ms()
    if (self._approach_deadline and
        ticks_diff(now, self._approach_deadline) > 0):
      self._skip_or_home("FACE/FINAL total timeout")
      return
    if ticks_diff(now, self._phase_ms) > int(self._cfg.final_approach_timeout_ms):
      self._skip_or_home("FINAL_APPROACH timeout")
      return
    target = sensors.get("target") if sensors else None
    if target is None:
      self._vision_lost_tick(sensors, "FINAL_APPROACH target lost")
      return
    if sensors and sensors.get("new_frame"):
      self._vision_lost = 0
    if abs(self._yaw_err(self._yaw_target)) > (
        float(self._cfg.orbit_yaw_tol_deg) * 1.5):
      self._enter_face(self._yaw_target)
      return
    y2 = float(target[9])
    cx = float(target[6])
    if y2 >= float(self._cfg.tracking.contact_bottom_pct):
      if self._push_cx_ok(cx):
        self._hold_brake()
        self._enter_push()
      else:
        info("MATCH", "final cx=%.1f off → FACE" % cx)
        self._enter_face(self._yaw_target)
      return
    bearing = (cx - 50.0) / 50.0
    rot = (float(self._cfg.tracking.bearing_actuation_sign) *
           self._bearing_pid.update(bearing, self._control_dt()))
    self._write_vector(
      float(self._cfg.tracking.final_approach_speed), 0.0, rot)

  def _push_watch_frame(self, sensors, elapsed):
    if not sensors or not sensors.get("new_frame"):
      return None
    need = int(self._cfg.push_watch_frames)
    t = sensors.get("target")
    if t is None:
      if elapsed >= int(self._cfg.push_lost_blind_ms):
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
    if self._push_cx_ok(cx):
      self._push_bad = 0
      self._push_bad_kind = ""
      return "ok"
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
