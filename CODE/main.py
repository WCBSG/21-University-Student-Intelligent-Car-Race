"""
main.py — RT1021 智能车主程序

DEBUG: 先 import Menu，再开 LCD，最后才建 TCS/Match（避免 Menu OOM）
MATCH: 无屏无 Menu；上电按住 C20≥1s 或 boot 文件
"""

from machine import Pin, UART
from time import sleep_ms, ticks_ms, ticks_diff
import gc

def _log(msg, tag):
  print("[%s] %s" % (tag, msg))

def _mem(tag):
  gc.collect()
  print("[MEM] %s free=%d" % (tag, gc.mem_free()))

# =============================================================================
LED = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
C20 = Pin('C20', Pin.IN, pull=Pin.PULL_UP_47K)

from boot_mode import resolve_boot_mode, request_reboot, clear_boot_file
BOOT_MODE = resolve_boot_mode(C20, hold_ms=1000)
_log("Boot mode: %s (C20=%d)" % (BOOT_MODE, C20.value()), "INIT")

# =============================================================================
_log("Motors...", "INIT")
from Motor import MotionControl
motors = MotionControl()
from ctrl.arbiter import MotorArbiter
arbiter = MotorArbiter(motors)

_log("Config...", "INIT")
from config import config as cfg, load_config
load_config()

_log("IMU963RA...", "INIT")
from imu import ImuSensor
imu = ImuSensor(calibrate_samples=100, beta=0.05, model="660")
_log("IMU OK model=%s (calibrating...)" % imu.model, "INIT")
_mem("after imu")

from smartcar import ticker
tickCount = 0

def onTick(_):
  global tickCount
  tickCount += 1
  imu.update()
  if tickCount >= 100:
    tickCount = 0
    gc.collect()

tkr = ticker(1)
tkr.capture_list(imu.raw)
tkr.callback(onTick)
tkr.start(10)

has_target = False
target = None
y2 = 0.0
tcs = None
match = None
camera = None
robot = None

def _build_sensors():
  global has_target, target, y2
  new_frame = False
  cam_timeout = False
  if camera is not None and camera.is_ready:
    frame = camera.poll()
    if frame is not None:
      new_frame = True
      has_target = frame.has_target
      if has_target:
        target = select_target(frame.detections, cfg)
        has_target = target is not None
        if has_target:
          y2 = target[9]
        else:
          target = None
          y2 = 0.0
      else:
        target = None
        y2 = 0.0
    cam_timeout = camera.timed_out
  else:
    has_target = False
    target = None
    y2 = 0.0
    cam_timeout = False

  tcs_crossed = False
  tcs_yellow = False
  if tcs is not None:
    tcs_crossed = tcs.crossed_yellow()
    tcs_yellow = tcs._prev_yellow

  return {
    "new_frame": new_frame,
    "has_target": has_target,
    "target": target,
    "y2": y2,
    "cam_timeout": cam_timeout,
    "tcs_crossed": tcs_crossed,
    "tcs_yellow": tcs_yellow,
  }

# =============================================================================
#                      MATCH（无 Menu / 无 LCD）
# =============================================================================
if BOOT_MODE == "MATCH":
  from app.fsm import build_robot
  from link.camera_rx import CameraRx
  from ctrl.track import select_target
  from sensors.tcs3472 import TCS3472, make_i2c
  from match.runner import MatchRunner

  camera = CameraRx(UART(5, baudrate=460800), timeout_ms=cfg.tracking.cam_timeout_ms)
  tcs = TCS3472(make_i2c())
  robot = build_robot(arbiter, cfg, imu)
  match = MatchRunner(robot, arbiter, tcs, cfg)
  _log("MATCH profile", "MATCH")

  M_WAIT_CALIB, M_WAIT_CAM, M_READY, M_RUN, M_DONE = 0, 1, 2, 3, 4
  mphase = M_WAIT_CALIB
  mphase_ms = ticks_ms()
  ready_countdown = 3
  c20_last = 1
  c20_down_ms = 0

  while C20.value() == 0:
    sleep_ms(20)
  c20_last = 1
  c20_armed = True

  LED.low()
  _last_ms = ticks_ms()
  _loop = 0

  while True:
    now = ticks_ms()
    dt = ticks_diff(now, _last_ms) / 1000.0
    if dt <= 0.0 or dt > 0.5:
      dt = 0.02
    _last_ms = now
    _loop += 1

    c20_now = C20.value()
    if c20_armed:
      if not c20_now and c20_last:
        c20_down_ms = now
      elif not c20_now and c20_down_ms:
        if mphase == M_DONE and ticks_diff(now, c20_down_ms) >= 2000:
          request_reboot("DEBUG")
      elif c20_now and not c20_last and c20_down_ms:
        held = ticks_diff(now, c20_down_ms)
        if mphase == M_RUN and held < 2000:
          match.stop()
          mphase = M_DONE
          mphase_ms = now
        elif mphase == M_DONE and held < 2000:
          if match.start():
            mphase = M_RUN
            mphase_ms = now
        c20_down_ms = 0
    c20_last = c20_now

    if mphase == M_WAIT_CALIB:
      LED.value(0 if (now % 400) < 200 else 1)
      if imu.is_calibrated:
        mphase = M_WAIT_CAM
        mphase_ms = now
        _log("Calibrated → handshake...", "MATCH")
    elif mphase == M_WAIT_CAM:
      LED.value(0 if (now % 1000) < 500 else 1)
      if camera.handshake(retries=1, retry_ms=50):
        camera.set_ready()
        mphase = M_READY
        mphase_ms = now
        ready_countdown = 3
        _log("Camera OK → READY", "MATCH")
    elif mphase == M_READY:
      LED.value(0)
      elapsed = ticks_diff(now, mphase_ms) / 1000.0
      countdown = max(0, 3 - int(elapsed))
      if countdown != ready_countdown and countdown > 0:
        ready_countdown = countdown
        _log("%d..." % countdown, "MATCH")
      if elapsed >= 3.0:
        match.start()
        mphase = M_RUN
        _log("GO!", "MATCH")
    elif mphase == M_RUN:
      LED.value(0)
      if not match.is_running and match.phase == "DONE":
        mphase = M_DONE
        mphase_ms = now
        _log("DONE scored=%d" % match.scored_count, "MATCH")
    elif mphase == M_DONE:
      cycle = (now - mphase_ms) % 1200
      LED.value(0 if (cycle < 150 or (300 < cycle < 450) or (600 < cycle < 750)) else 1)

    sensors = _build_sensors()
    robot.tick(dt, sensors)
    match.tick(dt, sensors)
    if _loop % 100 == 0:
      print("[MATCH] phase=%d free=%d" % (mphase, gc.mem_free()))
    sleep_ms(20)

# =============================================================================
#                      DEBUG：先编译 Menu，再开 LCD，最后 TCS/Match
# =============================================================================
_log("DEBUG profile", "INIT")

_log("Import Menu (before LCD)...", "INIT")
gc.collect()
from Menu import MenuInit
from app.intent import IntentQueue, ABORT
from app.fsm import build_robot, IDLE
from link.camera_rx import CameraRx
from ctrl.track import select_target
_mem("after Menu import")

intents = IntentQueue()

_log("Display...", "INIT")
from display import LCD_Drv, LCD
# IPS200：CS 必须先脉冲一次并保持低，否则屏不亮/无显示（见 E5_01 例程）
_cs  = Pin('B29', Pin.OUT, value=True)
_cs.high()
_cs.low()
_dc  = Pin('B5',  Pin.OUT, value=True)
_rst = Pin('B31', Pin.OUT, value=True)
_blk = Pin('C21', Pin.OUT, value=True)  # 背光开
_lcd_drv = LCD_Drv(SPI_INDEX=2, BAUDRATE=60000000,
                   DC_PIN=_dc, RST_PIN=_rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
_lcd = LCD(_lcd_drv)
_lcd.mode(1)
_lcd.color(0xFFFF, 0x0000)
_lcd.clear(0x0000)
_lcd.str24(40, 80, "LCD OK", 0x07E0)
_mem("after lcd")

camera = CameraRx(UART(5, baudrate=460800), timeout_ms=cfg.tracking.cam_timeout_ms)
sleep_ms(500)
_log("Connecting to camera...", "CAM")
_dots = ["", ".", "..", "..."]
for retry in range(1, 21):
  _lcd.clear(0x0000)
  _lcd.str24(20, 40, "Wait Camera Connect" + _dots[(retry - 1) % 4], 0xFFFF)
  _lcd.str24(60, 80, str(retry) + "/20", 0x07E0)
  if camera.handshake(retries=1, retry_ms=100):
    _log("Connected after %d retries" % retry, "CAM")
    break
  if camera.failed:
    _log("Camera self-test FAILED", "CAM")
    break
else:
  _log("Camera handshake TIMEOUT", "CAM")
gc.collect()

robot = build_robot(arbiter, cfg, imu)
_mem("after fsm")

menu = None
_log("MenuInit...", "INIT")
gc.collect()
try:
  menu = MenuInit(
    W=320, H=200, imu=imu, hdg=None, tracker=None, camera=camera,
    intents=intents, robot=robot,
    _lcd=_lcd, _lcd_drv=_lcd_drv,
  )
  _log("Menu OK", "INIT")
except MemoryError as e:
  _log("MenuInit MemoryError: %s" % e, "INIT")
  _lcd.clear(0x0000)
  _lcd.str16(10, 40, "Menu OOM — ENTER=Match", 0xFFE0)
  _lcd.str16(10, 70, "C20 hold2s=MATCH", 0xFFFF)
_mem("after menu")

# Menu 起来后再加载 TCS / Match（省编译期峰值）
_log("TCS+Match...", "INIT")
gc.collect()
from sensors.tcs3472 import TCS3472, make_i2c
from match.runner import MatchRunner
tcs = TCS3472(make_i2c())
match = MatchRunner(robot, arbiter, tcs, cfg)
_mem("after tcs+match")

UP = Pin('C8', Pin.IN, pull=Pin.PULL_UP_47K)
DOWN = Pin('C9', Pin.IN, pull=Pin.PULL_UP_47K)
ENTER = Pin('C14', Pin.IN, pull=Pin.PULL_UP_47K)
BACK = Pin('C15', Pin.IN, pull=Pin.PULL_UP_47K)

# TCS/Match 占 RAM 后再强制刷一帧（避免 clear 后未画完导致黑屏）
if menu is not None:
  gc.collect()
  menu._dirty = True
  menu.update_display()
sleep_ms(100)
LED.low()
_log("Main loop running", "Main")

keylast = [1, 1, 1, 1]
c20_press_ms = 0
c20_last = 1
c20_fired = False
_last_ms = ticks_ms()
_last_dbg_ms = ticks_ms()
_loop_cnt = 0

while True:
  now = ticks_ms()
  dt = ticks_diff(now, _last_ms) / 1000.0
  if dt <= 0.0 or dt > 0.5:
    dt = 0.02
  _last_ms = now
  _loop_cnt += 1

  # -- C20 长按 2s → MATCH --
  c20_now = C20.value()
  if not c20_now and c20_last:
    c20_press_ms = now
    c20_fired = False
  elif not c20_now:
    if (not c20_fired) and ticks_diff(now, c20_press_ms) >= 2000:
      c20_fired = True
      request_reboot("MATCH")
  elif c20_now:
    c20_press_ms = 0
    c20_fired = False
  c20_last = c20_now

  if ticks_diff(now, _last_dbg_ms) >= 2000:
    _last_dbg_ms = now
    print("[DBG] loop=%d robot=%s match=%s menu=%s free=%d" % (
      _loop_cnt, robot.state, match.phase, menu is not None, gc.mem_free()))

  if not BACK.value() and keylast[3]:
    if match.is_running or match.phase == "DONE":
      match.stop()
    else:
      intents.post(ABORT)
    if menu is not None:
      menu.handle_input('BACK')
  if not UP.value() and keylast[0] and menu is not None:
    menu.handle_input('UP')
  if not DOWN.value() and keylast[1] and menu is not None:
    menu.handle_input('DOWN')
  if not ENTER.value() and keylast[2]:
    if match.phase in ("IDLE", "DONE") and robot.state == IDLE and menu is None:
      match.start()
    elif menu is not None:
      menu.handle_input('ENTER')
  keylast = [UP.value(), DOWN.value(), ENTER.value(), BACK.value()]

  if len(intents) > 0:
    robot.drain_and_handle(intents)

  if robot.reconnect_pending:
    if camera.handshake(retries=30, retry_ms=30):
      robot.reconnect_pending = False
      camera.set_ready()
    else:
      robot.reconnect_pending = False

  sensors = _build_sensors()
  robot.tick(dt, sensors)
  match.tick(dt, sensors)

  if menu is not None:
    menu.update_display()
  sleep_ms(20)
