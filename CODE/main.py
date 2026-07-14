"""
main.py — RT1021 智能车主程序

Init: 软件 import → LCD → Camera → FSM → TCS → Match → Menu(可选) → Loop
P1: MatchRunner 单件闭环；ENTER 发车，BACK 急停。
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
#                               Motors + Config + IMU
# =============================================================================
_log("Motors...", "INIT")
from Motor import MotionControl
motors = MotionControl()
from ctrl.arbiter import MotorArbiter
arbiter = MotorArbiter(motors)
_log("Motors+Arbiter OK", "INIT")

_log("Config...", "INIT")
from config import config as cfg, load_config
load_config()
_log("Config OK", "INIT")

_log("IMU660RX...", "INIT")
from imu import ImuSensor
imu = ImuSensor(calibrate_samples=100, beta=0.05)
_log("IMU660RX OK (calibrating...)", "INIT")
_mem("after imu")

# =============================================================================
#                     软件模块提前 import（LCD 帧缓冲前）
# =============================================================================
_log("Imports...", "INIT")
from app.intent import IntentQueue, ABORT
from app.fsm import build_robot, IDLE
from link.camera_rx import CameraRx
from ctrl.track import select_target
from sensors.tcs3472 import TCS3472, make_i2c
from match.runner import MatchRunner
_mem("after imports")

# =============================================================================
#                               Display
# =============================================================================
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
_log("Display OK", "INIT")
_mem("after lcd")

# =============================================================================
#                               Camera
# =============================================================================
_log("Camera UART5...", "INIT")
cam_uart = UART(5, baudrate=460800)
camera = CameraRx(cam_uart, timeout_ms=cfg.tracking.cam_timeout_ms)
_log("Camera UART5 OK", "INIT")

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

# =============================================================================
#                               FSM + TCS + Match
# =============================================================================
_log("FSM...", "INIT")
intents = IntentQueue()
robot = build_robot(arbiter, cfg, imu)
_log("FSM OK", "INIT")

_log("TCS3472 I2C1...", "INIT")
tcs = TCS3472(make_i2c())
_log("TCS3472 OK", "INIT")

_log("Match...", "INIT")
match = MatchRunner(robot, arbiter, tcs, cfg)
_log("Match OK", "INIT")
_mem("after match")

# =============================================================================
#                               Menu（OOM 时跳过，用按键跑 Match）
# =============================================================================
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
  _lcd.str16(10, 40, "No Menu — ENTER=Match", 0xFFE0)
  _lcd.str16(10, 70, "BACK=Abort", 0xFFFF)
_mem("after menu")

# =============================================================================
#                               Buttons / Ticker
# =============================================================================
LED = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
UP    = Pin('C8',  Pin.IN, pull=Pin.PULL_UP_47K)
DOWN  = Pin('C9',  Pin.IN, pull=Pin.PULL_UP_47K)
ENTER = Pin('C14', Pin.IN, pull=Pin.PULL_UP_47K)
BACK  = Pin('C15', Pin.IN, pull=Pin.PULL_UP_47K)

_log("Ticker...", "INIT")
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
_log("Ticker OK", "INIT")

_log("Init Complete", "INIT")
if menu is not None:
  _lcd.clear(0x0000)
sleep_ms(100)
LED.low()
_log("Main loop — ENTER start Match, BACK abort", "Main")
keylast = [1, 1, 1, 1]
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

  # ---- Debug Heartbeat (每 2 秒) ----
  if ticks_diff(now, _last_dbg_ms) >= 2000:
    _last_dbg_ms = now
    print("[DBG] loop=%d robot=%s match=%s free=%d" % (
      _loop_cnt, robot.state, match.phase, gc.mem_free()))

  # ---- Keys ----
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
    # P1: IDLE 时 ENTER 启 Match；有 Menu 时仍可进菜单项
    if match.phase in ("IDLE", "DONE") and robot.state == IDLE:
      match.start()
    elif menu is not None:
      menu.handle_input('ENTER')
  keylast = [UP.value(), DOWN.value(), ENTER.value(), BACK.value()]

  # ---- drain Intent ----
  if len(intents) > 0:
    robot.drain_and_handle(intents)

  # ---- RECONNECT ----
  if robot.reconnect_pending:
    if camera.handshake(retries=30, retry_ms=30):
      robot.reconnect_pending = False
      camera.set_ready()
    else:
      robot.reconnect_pending = False

  # ---- Camera → sensors ----
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

  # ---- TCS（每拍一次 crossed，供 Match PUSH）----
  tcs_crossed = tcs.crossed_yellow()
  tcs_yellow = tcs._prev_yellow

  sensors = {
    "new_frame": new_frame,
    "has_target": has_target,
    "target": target,
    "y2": y2,
    "cam_timeout": cam_timeout,
    "tcs_crossed": tcs_crossed,
    "tcs_yellow": tcs_yellow,
  }

  # ---- 控制：先 robot，再 match（PUSH 时 Match 抢 Arbiter）----
  robot.tick(dt, sensors)
  match.tick(dt, sensors)

  if menu is not None:
    menu.update_display()
  sleep_ms(20)
