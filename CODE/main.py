"""
main.py — 纯比赛固件（无 Menu / 无 LCD）

上电：Motors → Config → IMU → Camera → TCS → FSM → Match
等待 IMU 标定 + 相机握手完成后串口提示 READY
短按 C20：发车 / 急停 / DONE 后再开一局
状态全部 print 到串口
"""

from machine import Pin, UART
from time import sleep_ms, ticks_ms, ticks_diff
import gc

LED = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
C20 = Pin('C20', Pin.IN, pull=Pin.PULL_UP_47K)


def _log(msg, tag="MAIN"):
  print("[%s] %s" % (tag, msg))


def _mem(tag):
  gc.collect()
  print("[MEM] %s free=%d" % (tag, gc.mem_free()))


# =============================================================================
#                              硬件 / 模块初始化
# =============================================================================
_log("Motors...")
from Motor import MotionControl
motors = MotionControl()
from ctrl import MotorArbiter
arbiter = MotorArbiter(motors)

_log("Config...")
from config import config as cfg, load_config
load_config()

_log("IMU963...")
from imu import ImuSensor
imu = ImuSensor(calibrate_samples=100, beta=0.05, model="963")
imu.set_mag_offset(cfg.mag_ox, cfg.mag_oy, cfg.mag_oz)
imu.mag_enabled = cfg.mag_enabled
_log("IMU model=%s mag=%s calibrating..." % (
  imu.model, "ON" if imu.mag_enabled else "OFF"), "IMU")
_mem("imu")

from smartcar import ticker
_tick_n = 0


def _on_tick(_):
  global _tick_n
  _tick_n += 1
  imu.update()
  if _tick_n >= 100:
    _tick_n = 0
    gc.collect()


tkr = ticker(1)
tkr.capture_list(imu.raw)
tkr.callback(_on_tick)
tkr.start(10)

_log("Camera UART5...")
from camera import CameraRx
from ctrl import select_target
camera = CameraRx(UART(5, baudrate=460800), timeout_ms=cfg.tracking.cam_timeout_ms)

_log("TCS...")
from tcs3472 import TCS3472, make_i2c
tcs = TCS3472(make_i2c())

_log("FSM + Match...")
from fsm import build_robot
from runner import MatchRunner
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


def _build_sensors():
  global _has_target, _target, _y2, _filt_ms
  new_frame = False
  cam_timeout = False
  if camera.is_ready:
    frame = camera.poll()
    if frame is not None:
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
            print("[CAM] filtered n=%d cls=%d sc=%d want=%s allow=%s" % (
              raw_n, int(d0[0]), int(d0[1]),
              cfg.tracking.target_class, getattr(cfg, "match_allow", None)))
      else:
        _target = None
        _y2 = 0.0
    cam_timeout = camera.timed_out
  else:
    _has_target = False
    _target = None
    _y2 = 0.0

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
# 等松开上电时可能按住的 C20
while C20.value() == 0:
  sleep_ms(20)

_log("Wait IMU calib...", "BOOT")
LED.high()
while not imu.is_calibrated:
  LED.value(0 if (ticks_ms() % 400) < 200 else 1)
  sleep_ms(20)
LED.low()
_log("IMU calibrated yaw=%.1f" % imu.get_yaw(), "BOOT")

_log("Camera handshake...", "BOOT")
_hs = False
for retry in range(1, 41):
  LED.value(0 if (ticks_ms() % 1000) < 500 else 1)
  if camera.handshake(retries=1, retry_ms=80):
    camera.set_ready()
    _hs = True
    _log("Camera OK (try %d)" % retry, "BOOT")
    break
  if camera.failed:
    _log("Camera self-test FAIL", "BOOT")
    break
  sleep_ms(50)
if not _hs:
  _log("Camera handshake TIMEOUT — start is locked", "BOOT")
else:
  camera.set_ready()

LED.low()
_log("======== READY: short-press C20 to START ========", "BOOT")
_log("layout=%d N=%d duty=%.0f" % (
  cfg.start_layout, cfg.match_target_count, cfg.drive_duty), "BOOT")
_mem("loop")

# =============================================================================
#                              主循环
# =============================================================================
# 相位: IDLE(等C20) | RUN | DONE
phase = "IDLE"
c20_last = 1
c20_down_ms = 0
_last_ms = ticks_ms()
_last_stat_ms = ticks_ms()
_loop = 0
_last_match_phase = ""

while True:
  now = ticks_ms()
  dt = ticks_diff(now, _last_ms) / 1000.0
  if dt <= 0.0 or dt > 0.5:
    dt = 0.02
  _last_ms = now
  _loop += 1

  # —— C20：按下沿记时，松开边沿触发 ——
  c20_now = C20.value()
  if not c20_now and c20_last:
    c20_down_ms = now
  elif c20_now and not c20_last and c20_down_ms:
    held = ticks_diff(now, c20_down_ms)
    if held < 2000:
      if phase == "RUN":
        match.stop()
        phase = "IDLE"
        _log("ABORT by C20 → IDLE", "MATCH")
      elif phase in ("IDLE", "DONE"):
        if camera.failed:
          _log("Camera FAILED — cannot start", "MATCH")
        elif not camera.is_ready:
          _log("Camera not ready — cannot start", "MATCH")
        elif match.start():
          phase = "RUN"
          _log("GO! scored reset", "MATCH")
        else:
          _log("start refused phase=%s" % match.phase, "MATCH")
    c20_down_ms = 0
  c20_last = c20_now

  # —— 业务 ——
  sensors = _build_sensors()
  imu._motor_on = arbiter.motors_active
  robot.tick(dt, sensors)
  match.tick(dt, sensors)

  if phase == "RUN":
    LED.low()
    if match.phase != _last_match_phase:
      _last_match_phase = match.phase
      _log("%s robot=%s scored=%d" % (
        match.info, robot.state, match.scored_count), "MATCH")
    if not match.is_running and match.phase == "DONE":
      phase = "DONE"
      _log("DONE scored=%d — C20 to run again" % match.scored_count, "MATCH")
  elif phase == "DONE":
    # 闪烁提示完赛
    LED.value(0 if ((now // 150) % 8) < 3 else 1)
  else:
    # IDLE 慢闪：可发车
    LED.value(0 if (now % 1000) < 50 else 1)

  if ticks_diff(now, _last_stat_ms) >= 2000:
    _last_stat_ms = now
    yaw = imu.get_yaw()
    ht = "Y" if sensors.get("has_target") else "N"
    yl = "Y" if sensors.get("tcs_on_line") else "N"
    print("[STAT] %s match=%s robot=%s yaw=%+.1f tgt=%s line=%s free=%d" % (
      phase, match.info, robot.state, yaw, ht, yl, gc.mem_free()))

  sleep_ms(20)
