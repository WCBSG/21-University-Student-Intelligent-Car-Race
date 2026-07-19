from machine import Pin, UART
from time import sleep_ms, ticks_ms, ticks_diff
import gc
from log import info, flush as log_flush
LED = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
C20 = Pin('C20', Pin.IN, pull=Pin.PULL_UP_47K)
_gc_last_ms = 0
_gc_need_ok = False
def _gc_maybe():
  global _gc_last_ms, _gc_need_ok
  free = gc.mem_free()
  if free >= 24576:
    _gc_need_ok = False
  if free >= 8192:
    return False
  now = ticks_ms()
  if free >= 3072:
    if _gc_need_ok or ticks_diff(now, _gc_last_ms) < 200:
      return False
  gc.collect()
  _gc_last_ms = now
  _gc_need_ok = gc.mem_free() < 24576
  return True
info("MAIN", "Motors...")
from motion import MotionControl, MotorArbiter, select_target
motors = MotionControl()
arbiter = MotorArbiter(motors)
info("MAIN", "Config...")
import config as cfg
info("MAIN", "Match import...")
gc.collect()
from match import MatchRunner
gc.collect()
info("MAIN", "IMU963...")
from imu import ImuSensor
imu = ImuSensor(calibrate_samples=int(cfg.imu_calibrate_samples),
                beta=float(cfg.imu_beta), model="963")
imu.set_mag_offset(cfg.mag_ox, cfg.mag_oy, cfg.mag_oz)
imu.mag_enabled = cfg.mag_enabled
imu.set_fusion_params(
  gyro_still=float(cfg.imu_gyro_still),
  acc_still=float(cfg.imu_acc_still),
  bias_alpha=float(cfg.imu_bias_alpha),
  mag_alpha=float(cfg.imu_mag_alpha),
  mag_dead=float(cfg.imu_mag_dead),
  mag_pull_max=float(cfg.imu_mag_pull_max),
  mag_still_need=int(cfg.imu_mag_still_need),
  still_needed=int(cfg.imu_still_needed),
  mag_lpf_alpha=float(cfg.imu_mag_lpf_alpha),
  gyro_scale=float(cfg.imu_gyro_scale),
  spin_beta=float(cfg.imu_spin_beta),
  spin_dps=float(cfg.imu_spin_dps))
info("IMU", "IMU model=%s mag=%s scale=%.3f calibrating..." % (
  imu.model, "ON" if imu.mag_enabled else "OFF", float(cfg.imu_gyro_scale)))
gc.collect()
from smartcar import ticker
_imu_err_n = 0
_tcs_on_line = False
_tcs_last_ms = 0
_tcs_err_n = 0
def _on_tick_imu(_):
  global _imu_err_n
  try:
    imu.update()
  except Exception as e:
    _imu_err_n += 1
    if _imu_err_n <= 3:
      info("IMU", "update err: %s" % e)
tkr_imu = ticker(1)
tkr_imu.capture_list(imu.raw)
tkr_imu.callback(_on_tick_imu)
tkr_imu.start(5)
info("MAIN", "Camera UART5...")
from camera import CameraRx
camera = CameraRx(
  UART(5, baudrate=460800),
  timeout_ms=cfg.tracking_cam_timeout_ms,
  poll_max_frames=cfg.camera_poll_max_frames,
  poll_budget_ms=cfg.camera_poll_budget_ms)
info("MAIN", "TCS...")
from tcs3472 import TCS3472, make_i2c
tcs = TCS3472(make_i2c())
tcs.confirm_n = int(cfg.tcs_confirm_n)
tcs.yellow_r_min = float(cfg.tcs_r_min)
tcs.yellow_g_min = float(cfg.tcs_g_min)
tcs.yellow_b_max = float(cfg.tcs_b_max)
tcs.yellow_c_min = int(cfg.tcs_c_min)
gc.collect()
info("MAIN", "Match build...")
gc.collect()
match = MatchRunner(arbiter, tcs, cfg, imu)
gc.collect()
def _poll_tcs(now):
  global _tcs_on_line, _tcs_last_ms, _tcs_err_n
  if ticks_diff(now, _tcs_last_ms) < int(cfg.tcs_poll_ms):
    return
  _tcs_last_ms = now
  try:
    tcs.confirm_n = (
      int(cfg.tcs_push_confirm_n)
      if match.phase == "PUSH" else int(cfg.tcs_confirm_n))
    tcs.sample()
    _tcs_on_line = bool(tcs.on_line)
    match.check_field_lock(_tcs_on_line, now)
  except Exception as e:
    _tcs_err_n += 1
    _tcs_on_line = False
    if _tcs_err_n <= 3 or _tcs_err_n % 100 == 0:
      info("TCS", "sample err x%d: %s" % (_tcs_err_n, e))
_has_target = False
_target = None
_y2 = 0.0
_filt_ms = 0
_last_frame_ms = 0
_sensors = {
  "new_frame": False,
  "has_target": False,
  "target": None,
  "y2": 0.0,
  "cam_timeout": False,
  "tcs_on_line": False,
  "brick_blocking": False,
  "brick": None,
  "suspect_target": None,
}
def _build_sensors():
  global _has_target, _target, _y2, _filt_ms, _last_frame_ms
  new_frame = False
  cam_timeout = False
  brick_target = None
  suspect_target = None
  if camera.is_ready:
    frame = camera.poll()
    if frame is not None:
      _last_frame_ms = ticks_ms()
      new_frame = True
      raw_n = frame.num
      _has_target = frame.has_target
      raw_dets = frame.detections
      if _has_target:
        if match.is_running:
          _target = select_target(
            raw_dets, cfg,
            allow=match.match_allow, target_class=match.filter_class)
        else:
          _target = select_target(raw_dets, cfg)
        _has_target = _target is not None
        if _has_target:
          _y2 = _target[9]
        else:
          _target = None
          _y2 = 0.0
          now = ticks_ms()
          if raw_n > 0 and ticks_diff(now, _filt_ms) > 1000:
            _filt_ms = now
            d0 = raw_dets[0]
            want = match.filter_class if match.is_running else cfg.tracking_target_class
            allow = match.match_allow if match.is_running else None
            info("CAM", "filtered n=%d cls=%d sc=%d want=%s allow=%s" % (
              raw_n, int(d0[0]), int(d0[1]), want, allow))
      else:
        _target = None
        _y2 = 0.0
        raw_dets = []
      if match.is_running and _target is None and raw_dets:
        allow = match.match_allow
        want = match.filter_class
        suspect_min = int(cfg.tracking_suspect_min_confidence)
        for d in raw_dets:
          cid = int(d[0])
          permitted = (
            cid in allow if allow is not None
            else (want == 7 or cid == int(want)))
          if (permitted and cid in (cfg.CLS_LEFT, cfg.CLS_UP, cfg.CLS_RIGHT)
              and int(d[1]) >= suspect_min):
            if suspect_target is None or d[1] > suspect_target[1]:
              suspect_target = d
      _brick_blocking = False
      if match.is_running and raw_dets:
        tcx = _target[6] if _target is not None else 50.0
        ty2 = _target[9] if _target is not None else float(cfg.path_brick_min_y2)
        for d in raw_dets:
          if (int(d[0]) == cfg.CLS_BRICK and
              int(d[1]) >= int(cfg.tracking_suspect_min_confidence)):
            closer = (
              d[9] > ty2 if _target is not None else d[9] >= ty2)
            if (abs(d[6] - tcx) < float(cfg.path_brick_cx_tol)
                and closer):
              if brick_target is None or d[9] > brick_target[9]:
                brick_target = d
        _brick_blocking = brick_target is not None
    else:
      if (_last_frame_ms and
          ticks_diff(ticks_ms(), _last_frame_ms) > int(cfg.tracking_stale_ms)):
        _has_target = False
        _target = None
        _y2 = 0.0
      _brick_blocking = False
    cam_timeout = camera.timed_out
  else:
    _has_target = False
    _target = None
    _y2 = 0.0
    _last_frame_ms = 0
    _brick_blocking = False
  on_line = _tcs_on_line
  _sensors["new_frame"] = new_frame
  _sensors["has_target"] = _has_target
  _sensors["target"] = _target
  _sensors["y2"] = _y2
  _sensors["cam_timeout"] = cam_timeout
  _sensors["tcs_on_line"] = on_line
  _sensors["brick_blocking"] = _brick_blocking
  _sensors["brick"] = brick_target
  _sensors["suspect_target"] = suspect_target
  return _sensors
_imu_ok = False
_cam_ok = False
while C20.value() == 0:
  sleep_ms(20)
info("BOOT", "Wait IMU calib...")
LED.high()
_imu_t0 = ticks_ms()
while not imu.is_calibrated:
  LED.value(0 if (ticks_ms() % 400) < 200 else 1)
  if ticks_diff(ticks_ms(), _imu_t0) > 10000:
    break
  sleep_ms(20)
LED.low()
_imu_ok = imu.is_calibrated
if _imu_ok:
  info("BOOT", "IMU calibrated yaw=%.1f" % imu.get_yaw())
else:
  info("BOOT", "IMU calibration TIMEOUT — fast LED blink")
info("BOOT", "Camera handshake...")
_hs = False
for retry in range(1, 41):
  LED.value(0 if (ticks_ms() % 1000) < 500 else 1)
  if camera.handshake(retries=1, retry_ms=80):
    _hs = True
    info("BOOT", "Camera OK (try %d)" % retry)
    break
  if camera.failed:
    info("BOOT", "Camera self-test FAIL")
    break
  sleep_ms(50)
_cam_ok = _hs
if not _hs:
  info("BOOT", "Camera handshake TIMEOUT — slow LED blink")
LED.low()
if _imu_ok and _cam_ok:
  info("BOOT", "======== READY: short-press C20 to START ========")
  info("BOOT", "layout=%d N=%d duty=%.0f" % (
    cfg.start_layout, cfg.match_target_count, cfg.drive_duty))
elif not _imu_ok:
  info("BOOT", "======== IMU FAILED — check sensor ========")
if not _cam_ok:
  info("BOOT", "======== CAMERA FAILED — start locked ========")
gc.collect()
phase = "IDLE"
c20_last = 1
c20_down_ms = 0
_last_ms = ticks_ms()
_last_stat_ms = ticks_ms()
_last_cal_ms = ticks_ms()
_last_tcs_log_ms = ticks_ms()
_loop = 0
_last_match_phase = ""
_cam_retry_ms = 0
while True:
  now = ticks_ms()
  dt = ticks_diff(now, _last_ms) / 1000.0
  if dt <= 0.0 or dt > 0.15:
    dt = 0.02
    if _loop > 10:
      info("MAIN", "WARN: dt=%.2fs clamped → 0.02" % (ticks_diff(now, _last_ms) / 1000.0))
  _last_ms = now
  _loop += 1
  c20_now = C20.value()
  if not c20_now and c20_last:
    c20_down_ms = now
  elif c20_now and not c20_last and c20_down_ms:
    held = ticks_diff(now, c20_down_ms)
    if held < 2000:
      if phase in ("RUN", "FAULT"):
        match.stop()
        phase = "IDLE"
        info("MATCH", "ABORT by C20 → IDLE")
      elif phase in ("IDLE", "DONE"):
        if not _imu_ok:
          info("MATCH", "IMU FAILED — cannot start")
        elif camera.failed:
          info("MATCH", "Camera FAILED — cannot start")
        elif not camera.is_ready:
          info("MATCH", "Camera not ready — cannot start")
        elif match.start():
          phase = "RUN"
          info("MATCH", "GO! scored reset")
        else:
          info("MATCH", "start refused phase=%s" % match.phase)
    c20_down_ms = 0
  c20_last = c20_now
  if not _cam_ok and ticks_diff(now, _cam_retry_ms) >= 100:
    _cam_retry_ms = now
    if camera.handshake(retries=1, retry_ms=80):
      _cam_ok = True
      info("BOOT", "Camera recovered!")
  _poll_tcs(now)
  sensors = _build_sensors()
  imu._motor_on = arbiter.motors_active
  match.tick(dt, sensors)
  if phase == "RUN":
    LED.low()
    if match.phase != _last_match_phase:
      _last_match_phase = match.phase
      info("MATCH", "%s scored=%d" % (match.status_text, match.scored_count))
    if not match.is_running and match.phase == "DONE":
      phase = "DONE"
      info("MATCH", "DONE scored=%d — C20 to run again" % match.scored_count)
    elif match.phase == "FAULT":
      phase = "FAULT"
      info("MATCH", "FAULT: %s — C20 to acknowledge" % match.fault_reason)
  elif phase == "DONE":
    LED.value(0 if ((now // 150) % 8) < 3 else 1)
  elif phase == "FAULT":
    _fault_slot = now % 1000
    LED.value(0 if (_fault_slot < 100 or 200 <= _fault_slot < 300) else 1)
  elif not _imu_ok:
    LED.value(0 if (now % 200) < 100 else 1)
  elif not _cam_ok:
    LED.value(0 if (now % 1000) < 500 else 1)
  else:
    LED.low()
  if ticks_diff(now, _last_cal_ms) >= 500:
    _last_cal_ms = now
    nav = match.navigation_snapshot(sensors)
    d0, d1, d2 = arbiter.duties
    t = sensors.get("target")
    cx = "%.0f" % t[6] if t else "-"
    y2 = "%.0f" % t[9] if t else "-"
    info("NAV", "%s/%s y=%+.0f e=%+.0f cx=%s y2=%s | %+.0f,%+.0f,%+.0f | %+.0f,%+.0f,%+.0f %s c=%d l=%d bk=%d" % (
      nav[0], nav[1] or "-", nav[2], nav[4], cx, y2,
      nav[7], nav[8], nav[9], d0, d1, d2, arbiter.owner or "-",
      nav[10], nav[11], int(sensors.get("brick_blocking", False))))
  if ticks_diff(now, _last_stat_ms) >= 3000:
    _last_stat_ms = now
    info("MEM", "free=%d" % gc.mem_free())
    log_flush()
  if (sensors.get("tcs_on_line") and
      ticks_diff(now, _last_tcs_log_ms) >= 300):
    _last_tcs_log_ms = now
    r, g, b, c, rn, gn, bn, ok = tcs.last_rgb()
    info("TCS", "ON R=%d G=%d B=%d C=%d ok=%s" % (r, g, b, c, ok))
  _gc_maybe()
  sleep_ms(5)
