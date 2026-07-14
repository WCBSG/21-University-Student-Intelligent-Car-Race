"""
main.py — RT1021 智能车主程序

启动: 读 boot_mode → DEBUG(现逻辑,Menu) / MATCH(精简,无屏,一键比赛)
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
#                              读启动模式
# =============================================================================
from boot_mode import read_and_clear, request_reboot
BOOT_MODE = read_and_clear()
_log("Boot mode: %s" % BOOT_MODE, "INIT")

# =============================================================================
#                     共用 Init（两种模式都需要）
# =============================================================================
_log("Motors...", "INIT")
from Motor import MotionControl
motors = MotionControl()
from ctrl.arbiter import MotorArbiter
arbiter = MotorArbiter(motors)

_log("Config...", "INIT")
from config import config as cfg, load_config
load_config()

_log("IMU660RX...", "INIT")
from imu import ImuSensor
imu = ImuSensor(calibrate_samples=100, beta=0.05)
_mem("after imu")

# 公共软件模块
from app.intent import IntentQueue, ABORT, START_TRACK
from app.fsm import build_robot, IDLE
from link.camera_rx import CameraRx
from ctrl.track import select_target
from sensors.tcs3472 import TCS3472, make_i2c
from match.runner import MatchRunner
_mem("after imports")

# Camera + TCS + FSM + Match（两种模式共用）
cam_uart = UART(5, baudrate=460800)
camera = CameraRx(cam_uart, timeout_ms=cfg.tracking.cam_timeout_ms)

tcs = TCS3472(make_i2c())
_log("TCS3472 OK", "INIT")

intents = IntentQueue()
robot = build_robot(arbiter, cfg, imu)
match = MatchRunner(robot, arbiter, tcs, cfg)
_log("FSM+Match OK", "INIT")

# LED + C20（板载）
LED = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
C20 = Pin('C20', Pin.IN, pull=Pin.PULL_UP_47K)

# Ticker
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

# =============================================================================
#                      MATCH 模式（精简，无屏/无Menu/无四键）
# =============================================================================
if BOOT_MODE == "MATCH":
  _log("MATCH profile — waiting calibration...", "MATCH")
  LED.low()
  sleep_ms(200)
  LED.high()

  # MATCH 阶段枚举
  M_WAIT_CALIB = 0
  M_WAIT_CAM   = 1
  M_READY      = 2
  M_RUN        = 3
  M_DONE       = 4

  mphase = M_WAIT_CALIB
  mphase_ms = ticks_ms()
  ready_countdown = 3
  led_period = 200  # ms, 用于闪烁
  c20_last = 1

  _log("MATCH loop running", "MATCH")
  sleep_ms(100)
  LED.low()

  _last_ms = ticks_ms()
  has_target = False
  target = None
  y2 = 0.0
  _loop = 0

  while True:
    now = ticks_ms()
    dt = ticks_diff(now, _last_ms) / 1000.0
    if dt <= 0.0 or dt > 0.5:
      dt = 0.02
    _last_ms = now
    _loop += 1

    # -- C20 急停（任意阶段）--
    c20_now = C20.value()
    if not c20_now and c20_last:
      if mphase == M_RUN:
        match.stop()
        mphase = M_DONE
        mphase_ms = now
      elif mphase == M_DONE:
        # 长按 C20 回 DEBUG
        pass  # 暂不实现，用 REPL 改
    c20_last = c20_now

    # -- LED --
    if mphase == M_WAIT_CALIB:
      LED.value(0 if (now % 400) < 200 else 1)   # 快闪 200ms
    elif mphase == M_WAIT_CAM:
      LED.value(0 if (now % 1000) < 500 else 1)  # 慢闪 500ms
    elif mphase == M_READY:
      LED.value(0)  # 常亮
    elif mphase == M_RUN:
      LED.value(0)  # 常亮
    elif mphase == M_DONE:
      # 三快闪模式
      cycle = (now - mphase_ms) % 1200
      LED.value(0 if (cycle < 150 or (300 < cycle < 450) or (600 < cycle < 750)) else 1)

    # -- 阶段流转 --
    if mphase == M_WAIT_CALIB:
      if imu.is_calibrated:
        mphase = M_WAIT_CAM
        mphase_ms = now
        _log("Calibrated → handshake...", "MATCH")

    elif mphase == M_WAIT_CAM:
      # 每拍一次握手尝试
      if camera.handshake(retries=1, retry_ms=50):
        camera.set_ready()
        mphase = M_READY
        mphase_ms = now
        ready_countdown = 3
        _log("Camera OK → READY in %ds" % ready_countdown, "MATCH")
      elif camera.failed:
        mphase = M_WAIT_CAM  # 重试，LED 持续慢闪
        mphase_ms = now
        _log("Camera self-test FAILED, retrying...", "MATCH")

    elif mphase == M_READY:
      elapsed = ticks_diff(now, mphase_ms) / 1000.0
      countdown = max(0, 3 - int(elapsed))
      if countdown != ready_countdown:
        ready_countdown = countdown
        if countdown > 0:
          _log("%d..." % countdown, "MATCH")
      if elapsed >= 3.0:
        match.start()
        mphase = M_RUN
        mphase_ms = now
        _log("GO!", "MATCH")

    elif mphase == M_RUN:
      if not match.is_running and match.phase == "DONE":
        mphase = M_DONE
        mphase_ms = now
        _log("DONE — scored=%d" % match.scored_count, "MATCH")

    # -- Camera + TCS sensors --
    new_frame = False
    cam_timeout = False
    if camera.is_ready:
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
            target = None; y2 = 0.0
        else:
          target = None; y2 = 0.0
      cam_timeout = camera.timed_out
    else:
      has_target = False; target = None; y2 = 0.0; cam_timeout = False

    tcs_crossed = tcs.crossed_yellow()
    tcs_yellow = tcs._prev_yellow

    sensors = {
      "new_frame": new_frame, "has_target": has_target,
      "target": target, "y2": y2, "cam_timeout": cam_timeout,
      "tcs_crossed": tcs_crossed, "tcs_yellow": tcs_yellow,
    }

    robot.tick(dt, sensors)
    match.tick(dt, sensors)

    # 心跳
    if _loop % 100 == 0:
      print("[MATCH] phase=%d free=%d" % (mphase, gc.mem_free()))

    sleep_ms(20)

# =============================================================================
#                      DEBUG 模式（有 Display + Menu）
# =============================================================================
else:
  _log("DEBUG profile", "INIT")

  # Display
  _log("Display...", "INIT")
  from display import LCD_Drv, LCD

  _dc  = Pin('B5',  Pin.OUT, value=True)
  _rst = Pin('B31', Pin.OUT, value=True)
  _blk = Pin('C21', Pin.OUT, value=True)

  _lcd_drv = LCD_Drv(SPI_INDEX=2, BAUDRATE=60000000,
                     DC_PIN=_dc, RST_PIN=_rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
  _lcd = LCD(_lcd_drv)
  _lcd.mode(1)
  _lcd.color(0xFFFF, 0x0000)
  _lcd.clear(0x0000)
  _mem("after lcd")

  # Camera 握手（同步，屏上有进度）
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
  gc.collect()

  # C20 DEBUG → 长按 2s 进 MATCH
  _log("C20 boot MATCH enabled", "INIT")

  # Menu
  menu = None
  _log("Menu...", "INIT")
  gc.collect()
  try:
    from Menu import MenuInit
    menu = MenuInit(
      W=320, H=200, imu=imu, hdg=None, tracker=None, camera=camera,
      intents=intents, robot=robot,
      _lcd=_lcd, _lcd_drv=_lcd_drv,
    )
    _log("Menu OK", "INIT")
  except MemoryError as e:
    _log("Menu SKIPPED (MemoryError: %s)" % e, "INIT")
    _lcd.clear(0x0000)
    _lcd.str16(10, 40, "No Menu — ENTER=Match BACK=Abort", 0xFFE0)
    _lcd.str16(10, 70, "C20 hold=MATCH mode", 0xFFFF)
  _mem("after menu")

  # 四键
  UP    = Pin('C8',  Pin.IN, pull=Pin.PULL_UP_47K)
  DOWN  = Pin('C9',  Pin.IN, pull=Pin.PULL_UP_47K)
  ENTER = Pin('C14', Pin.IN, pull=Pin.PULL_UP_47K)
  BACK  = Pin('C15', Pin.IN, pull=Pin.PULL_UP_47K)

  _log("Init Complete", "INIT")
  if menu is not None:
    _lcd.clear(0x0000)
  sleep_ms(100)
  LED.low()
  _log("Main loop — ENTER=Match BACK=Abort C20(hold)=MATCH", "Main")

  keylast = [1, 1, 1, 1]
  c20_press_ms = 0
  c20_last = 1
  _last_ms = ticks_ms()
  _last_dbg_ms = ticks_ms()
  _loop_cnt = 0
  has_target = False
  target = None
  y2 = 0.0

  while True:
    now = ticks_ms()
    dt = ticks_diff(now, _last_ms) / 1000.0
    if dt <= 0.0 or dt > 0.5:
      dt = 0.02
    _last_ms = now
    _loop_cnt += 1

    # -- C20 长按 2s → MATCH reboot --
    c20_now = C20.value()
    if not c20_now and c20_last:
      c20_press_ms = now
    elif not c20_now:
      if ticks_diff(now, c20_press_ms) >= 2000:
        _log("C20 hold 2s → MATCH reboot", "MAIN")
        request_reboot("MATCH")
    elif c20_now and not c20_last:
      c20_press_ms = 0
    c20_last = c20_now

    # -- Debug Heartbeat --
    if ticks_diff(now, _last_dbg_ms) >= 2000:
      _last_dbg_ms = now
      print("[DBG] loop=%d robot=%s match=%s free=%d" % (
        _loop_cnt, robot.state, match.phase, gc.mem_free()))

    # -- Keys --
    if not BACK.value() and keylast[3]:
      if match.is_running or match.phase == "DONE":
        match.stop()
      else:
        intents.post(ABORT)
      if menu is not None:
        menu.handle_input('BACK')
    if not UP.value() and keylast[0]:
      if menu is not None:
        menu.handle_input('UP')
    if not DOWN.value() and keylast[1]:
      if menu is not None:
        menu.handle_input('DOWN')
    if not ENTER.value() and keylast[2]:
      if match.phase in ("IDLE", "DONE") and robot.state == IDLE:
        match.start()
      elif menu is not None:
        menu.handle_input('ENTER')
    keylast = [UP.value(), DOWN.value(), ENTER.value(), BACK.value()]

    # -- drain Intent --
    if len(intents) > 0:
      robot.drain_and_handle(intents)

    # -- RECONNECT --
    if robot.reconnect_pending:
      if camera.handshake(retries=30, retry_ms=30):
        robot.reconnect_pending = False
        camera.set_ready()
      else:
        robot.reconnect_pending = False

    # -- Camera → sensors --
    new_frame = False
    cam_timeout = False
    if camera.is_ready:
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
            target = None; y2 = 0.0
        else:
          target = None; y2 = 0.0
      cam_timeout = camera.timed_out
    else:
      has_target = False; target = None; y2 = 0.0; cam_timeout = False

    tcs_crossed = tcs.crossed_yellow()
    tcs_yellow = tcs._prev_yellow

    sensors = {
      "new_frame": new_frame, "has_target": has_target,
      "target": target, "y2": y2, "cam_timeout": cam_timeout,
      "tcs_crossed": tcs_crossed, "tcs_yellow": tcs_yellow,
    }

    robot.tick(dt, sensors)
    match.tick(dt, sensors)

    if menu is not None:
      menu.update_display()
    sleep_ms(20)
