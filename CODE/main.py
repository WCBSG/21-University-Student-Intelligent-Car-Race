"""
main.py — RT1021 智能车主程序（步 4：robot.tick 唯一控制路径）

Init 顺序刻意把「纯软件 import」放在 LCD 帧缓冲之前，避免 Menu 编译 OOM。
"""

from machine import Pin
from time import sleep_ms, ticks_ms, ticks_diff
import gc

def _log(msg, tag):
  print("[%s] %s" % (tag, msg))

def _mem(tag):
  gc.collect()
  print("[MEM] %s free=%d" % (tag, gc.mem_free()))

# =============================================================================
#                               Init: Motors + Arbiter + Config + IMU
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
#                     软件模块提前 import（LCD 帧缓冲尚未分配）
# =============================================================================
_log("Imports...", "INIT")
from Menu import MenuInit
from app.intent import IntentQueue, ABORT
from app.fsm import build_robot
from link.camera_rx import CameraRx
from ctrl.track import select_target
_mem("after imports")

# =============================================================================
#                               Init: Display（帧缓冲吃 RAM）
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
#                               Init: Camera
# =============================================================================
_log("Camera UART5...", "INIT")
from machine import UART
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
#                               Init: FSM + Menu
# =============================================================================
_log("FSM...", "INIT")
intents = IntentQueue()
robot = build_robot(arbiter, cfg, imu)
_log("FSM OK", "INIT")
_mem("after fsm")

_log("Menu...", "INIT")
gc.collect()
menu = MenuInit(
  W=320, H=200, imu=imu, hdg=None, tracker=None, camera=camera,
  intents=intents, robot=robot,
  _lcd=_lcd, _lcd_drv=_lcd_drv,
)
_log("Menu OK", "INIT")
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
_lcd.clear(0x0000)
sleep_ms(100)
LED.low()
_log("Main loop running", "Main")
keylast = [1, 1, 1, 1]
_last_ms = ticks_ms()
# 帧间缓存：poll() 返回 None 时保持上一帧目标，避免 SEARCH 确认期误自旋
has_target = False
target = None
y2 = 0.0

while True:
  now = ticks_ms()
  dt = ticks_diff(now, _last_ms) / 1000.0
  if dt <= 0.0 or dt > 0.5:
    dt = 0.02
  _last_ms = now

  # ---- Keys → Intent / Menu ----
  if not BACK.value() and keylast[3]:
    intents.post(ABORT)
    menu.handle_input('BACK')
  if not UP.value() and keylast[0]:
    menu.handle_input('UP')
  if not DOWN.value() and keylast[1]:
    menu.handle_input('DOWN')
  if not ENTER.value() and keylast[2]:
    menu.handle_input('ENTER')
  keylast = [UP.value(), DOWN.value(), ENTER.value(), BACK.value()]

  # ---- drain Intent ----
  if len(intents) > 0:
    robot.drain_and_handle(intents)

  # ---- RECONNECT 先于 sensors（避免本拍仍带旧 cam_timeout → 立刻再 FAULT）----
  if robot.reconnect_pending:
    if camera.handshake(retries=30, retry_ms=30):
      robot.reconnect_pending = False
      camera.set_ready()
    else:
      robot.reconnect_pending = False

  # ---- Camera → sensors（保留上一帧 target，避免帧间抖动）----
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

  sensors = {
    "new_frame": new_frame,
    "has_target": has_target,
    "target": target,
    "y2": y2,
    "cam_timeout": cam_timeout,
  }

  robot.tick(dt, sensors)

  menu.update_display()
  sleep_ms(20)
