"""
main.py — 纯比赛固件（无 Menu / 无 LCD）

上电：Motors → Config → IMU → Camera → TCS → Match
等待 IMU 标定 + 相机握手完成后串口提示 READY
短按 C20：发车 / 急停 / DONE 后再开一局
状态全部 print 到串口
"""

from machine import Pin, UART
from time import sleep_ms, ticks_ms, ticks_diff
import gc
from log import info, setup as log_setup

LED = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
C20 = Pin('C20', Pin.IN, pull=Pin.PULL_UP_47K)


def _mem(tag):
  gc.collect()
  info("MEM", "%s free=%d" % (tag, gc.mem_free()))


# =============================================================================
#                              硬件 / 模块初始化
# =============================================================================
info("MAIN", "Motors...")
from motion import MotionControl, MotorArbiter, select_target
motors = MotionControl()
arbiter = MotorArbiter(motors)

info("MAIN", "Config...")
from config import config as cfg, load_config
load_config()
log_setup(cfg.debug_output)

# —— 提前 import 大模块（分步，避免单文件编译峰值 OOM） ——
# 实例化延后到所有硬件就绪之后
info("MAIN", "Robot + Match import...")
_mem("pre-robot")
from robot import build_robot
_mem("post-robot")
from match import MatchRunner
_mem("post-match")

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
  mag_still_need=int(cfg.imu_mag_still_need))
info("IMU", "IMU model=%s mag=%s calibrating..." % (
  imu.model, "ON" if imu.mag_enabled else "OFF"))
_mem("imu")

from smartcar import ticker

_imu_err_n = 0


def _on_tick(_):
  global _imu_err_n
  try:
    imu.update()
  except Exception as e:
    _imu_err_n += 1
    if _imu_err_n <= 3:
      info("IMU", "update err: %s" % e)


tkr = ticker(1)
tkr.capture_list(imu.raw)
tkr.callback(_on_tick)
tkr.start(10)

info("MAIN", "Camera UART5...")
from camera import CameraRx
camera = CameraRx(UART(5, baudrate=460800), timeout_ms=cfg.tracking.cam_timeout_ms)

info("MAIN", "TCS...")
from tcs3472 import TCS3472, make_i2c
tcs = TCS3472(make_i2c())
tcs.confirm_n = int(cfg.tcs_confirm_n)
tcs.yellow_r_min = float(cfg.tcs_r_min)
tcs.yellow_g_min = float(cfg.tcs_g_min)
tcs.yellow_b_max = float(cfg.tcs_b_max)
tcs.yellow_c_min = int(cfg.tcs_c_min)
_mem("tcs")

# —— 实例化（所有硬件依赖已就绪） ——
info("MAIN", "Match build...")
_mem("pre-build")
robot = build_robot(arbiter, cfg, imu)
match = MatchRunner(robot, arbiter, tcs, cfg)
_mem("ready")

# =============================================================================
#                              传感器打包
# =============================================================================
_has_target = False
_target = None
_y2 = 0.0
_filt_ms = 0
_ghost_n = 0


def _build_sensors():
  global _has_target, _target, _y2, _filt_ms, _ghost_n
  new_frame = False
  cam_timeout = False
  if camera.is_ready:
    frame = camera.poll()
    if frame is not None:
      _ghost_n = 0
      new_frame = True
      raw_n = frame.num
      _has_target = frame.has_target
      if _has_target:
        _target = select_target(frame.detections, cfg)
        _has_target = _target is not None
        if _has_target:
          _y2 = _target[9]
        else:
          _target = None
          _y2 = 0.0
          now = ticks_ms()
          if raw_n > 0 and ticks_diff(now, _filt_ms) > 1000:
            _filt_ms = now
            d0 = frame.detections[0]
            info("CAM", "filtered n=%d cls=%d sc=%d want=%s allow=%s" % (
              raw_n, int(d0[0]), int(d0[1]),
              cfg.tracking.target_class, getattr(cfg, "match_allow", None)))
      else:
        _target = None
        _y2 = 0.0
    else:
      _ghost_n += 1
      if _ghost_n >= 3:
        _has_target = False
        _target = None
        _y2 = 0.0
    cam_timeout = camera.timed_out
  else:
    _has_target = False
    _target = None
    _y2 = 0.0
    _ghost_n = 0

  crossed = False
  on_line = False
  if tcs is not None:
    crossed = tcs.crossed_yellow()
    on_line = tcs.on_line

  return {
    "new_frame": new_frame,
    "has_target": _has_target,
    "target": _target,
    "y2": _y2,
    "cam_timeout": cam_timeout,
    "tcs_crossed": crossed,
    "tcs_yellow": on_line,
    "tcs_on_line": on_line,
  }


# =============================================================================
#                              启动等待：标定 + 握手
# =============================================================================
_imu_ok = False
_cam_ok = False

# 等松开上电时可能按住的 C20
while C20.value() == 0:
  sleep_ms(20)

# —— IMU 标定（超时 10s） ——
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

# —— 摄像头握手 ——
info("BOOT", "Camera handshake...")
_hs = False
for retry in range(1, 41):
  LED.value(0 if (ticks_ms() % 1000) < 500 else 1)
  if camera.handshake(retries=1, retry_ms=80):
    camera.set_ready()
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
else:
  camera.set_ready()

LED.low()
if _imu_ok and _cam_ok:
  info("BOOT", "======== READY: short-press C20 to START ========")
  info("BOOT", "layout=%d N=%d duty=%.0f" % (
    cfg.start_layout, cfg.match_target_count, cfg.drive_duty))
elif not _imu_ok:
  info("BOOT", "======== IMU FAILED — check sensor ========")
if not _cam_ok:
  info("BOOT", "======== CAMERA FAILED — start locked ========")
_mem("loop")

# =============================================================================
#                              主循环
# =============================================================================
# 相位: IDLE(等C20) | RUN | FAULT | DONE
phase = "IDLE"
c20_last = 1
c20_down_ms = 0
_last_ms = ticks_ms()
_last_stat_ms = ticks_ms()
_last_cal_ms = ticks_ms()
_loop = 0
_gc_n = 0
_last_match_phase = ""
_cam_retry_ms = 0

while True:
  now = ticks_ms()
  dt = ticks_diff(now, _last_ms) / 1000.0
  if dt <= 0.0 or dt > 0.5:
    dt = 0.02
    if _loop > 10:  # 跳过启动阶段
      info("MAIN", "WARN: dt=%.2fs clamped → 0.02" % (ticks_diff(now, _last_ms) / 1000.0))
  _last_ms = now
  _loop += 1

  # —— C20：按下沿记时，松开边沿触发 ——
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

  # —— 摄像头后台重试（每 200ms） ——
  if not _cam_ok and ticks_diff(now, _cam_retry_ms) >= 200:
    _cam_retry_ms = now
    if camera.handshake(retries=1, retry_ms=80):
      camera.set_ready()
      _cam_ok = True
      info("BOOT", "Camera recovered!")

  # —— 业务 ——
  sensors = _build_sensors()
  imu._motor_on = arbiter.motors_active
  robot.tick(dt, sensors)
  match.tick(dt, sensors)

  if phase == "RUN":
    LED.low()
    if match.phase != _last_match_phase:
      _last_match_phase = match.phase
      info("MATCH", "%s robot=%s scored=%d" % (
        match.info, robot.state, match.scored_count))
    if not match.is_running and match.phase == "DONE":
      phase = "DONE"
      info("MATCH", "DONE scored=%d — C20 to run again" % match.scored_count)
    elif match.phase == "FAULT":
      phase = "FAULT"
      info("MATCH", "FAULT: %s — C20 to acknowledge" % match.fault_reason)
  elif phase == "DONE":
    # 闪烁提示完赛
    LED.value(0 if ((now // 150) % 8) < 3 else 1)
  elif phase == "FAULT":
    # 快速双闪：明确区别于 IDLE / DONE / 传感器启动失败。
    _fault_slot = now % 1000
    LED.value(0 if (_fault_slot < 100 or 200 <= _fault_slot < 300) else 1)
  elif not _imu_ok:
    # IMU 初始化失败：快闪 (周期 ~200ms)
    LED.value(0 if (now % 200) < 100 else 1)
  elif not _cam_ok:
    # 摄像头初始化失败：慢闪 (周期 ~1000ms)
    LED.value(0 if (now % 1000) < 500 else 1)
  else:
    # IDLE 长亮：等待发车
    LED.low()

  if ticks_diff(now, _last_stat_ms) >= 2000:
    _last_stat_ms = now
    yaw = imu.get_yaw()
    ht = "Y" if sensors.get("has_target") else "N"
    yl = "Y" if sensors.get("tcs_on_line") else "N"
    info("STAT", "%s match=%s robot=%s yaw=%+.1f tgt=%s line=%s free=%d" % (
     phase, match.info, robot.state, yaw, ht, yl, gc.mem_free()))

  if cfg.debug_output and ticks_diff(now, _last_cal_ms) >= 500:
    _last_cal_ms = now
    target = sensors.get("target")
    mx, my, mz = imu.mag_data
    yaw, src = imu.get_fused_yaw(motor_on=arbiter.motors_active, apply=False)
    mrel = imu.get_mag_rel()
    mrel_s = "n/a" if mrel is None else "%+.1f" % mrel
    m_on = "M" if arbiter.motors_active else "."
    if target is None:
      info("CAL", "yaw=%+.1f mrel=%s mag=(%d,%d,%d) src=%s %s target=NONE" % (
        yaw, mrel_s, mx, my, mz, src, m_on))
    else:
      info("CAL", "yaw=%+.1f mrel=%s mag=(%d,%d,%d) src=%s %s cls=%d score=%d cx=%.1f y2=%.1f area=%.1f" % (
        yaw, mrel_s, mx, my, mz, src, m_on, int(target[0]), int(target[1]),
        float(target[6]), float(target[9]), float(target[8])))
    nav = match.navigation_snapshot(sensors)
    d0, d1, d2 = arbiter.duties
    info("NAV", "phase=%s/%s yaw=%+.1f tgt=%+.1f err=%+.1f cx=%.1f y2=%.1f cmd=(%+.1f,%+.1f,%+.1f) pwm=(%+.1f,%+.1f,%+.1f) owner=%s conf=%d lost=%d" % (
      nav[0], nav[1] or "-", nav[2], nav[3], nav[4], nav[5], nav[6],
      nav[7], nav[8], nav[9], d0, d1, d2, arbiter.owner or "-",
      nav[10], nav[11]))
    tcs.debug_print()

  _gc_n += 1
  if _gc_n >= 100:
    _gc_n = 0
    _mem("loop")

  sleep_ms(20)
