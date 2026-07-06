"""
main.py — RT1021 智能车主程序

Init: Motors → IMU → HeadingController → Display → Camera Handshake → Menu → Ticker → Main Loop
"""

from machine import Pin
from time import sleep_ms, ticks_ms, ticks_diff
import gc
import sys

def _log(msg,sys):
  """Print debug message to console (UART REPL)"""
  print(f"[{sys}] {msg}")

# =============================================================================
#                               Init: Motors
# =============================================================================
_log("Motors...","INIT")
from Motor import MotionControl
motors = MotionControl()
_log("Motors OK","INIT")

# =============================================================================
#                               Init: IMU
# =============================================================================
_log("IMU660RX...","INIT")
from imu import ImuSensor
imu = ImuSensor(calibrate_samples=100, beta=0.05)
_log("IMU660RX OK (calibrating...)","INIT")

# =============================================================================
#                               Init: Heading
# =============================================================================
_log("HeadingController...","INIT")
from HeadingController import HeadingController
from config import config as cfg
hdg = HeadingController(motors, imu, cfg)
_log("HeadingController OK","INIT")

# =============================================================================
#                               Init: Display
# =============================================================================
_log("Display...","INIT")
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
_log("Display OK","INIT")

# =============================================================================
#                               Init: Camera Handshake
# =============================================================================
_log("Camera UART5...","INIT")
from machine import UART
cam_uart = UART(5, baudrate=460800)
from CameraReceiver import CameraReceiver
camera = CameraReceiver(cam_uart)
_log("Camera UART5 OK","INIT")

# 首次连接等待延迟 — 给 OpenART 启动时间
sleep_ms(500)

# 初始化时握手（20 次 × 100ms = 2 秒）
_log("Connecting to camera...","CAM")
_dots = ["", ".", "..", "..."]
for retry in range(1, 21):
  _lcd.clear(0x0000)
  _lcd.str24(20, 40, "Wait Camera Connect" + _dots[(retry - 1) % 4], 0xFFFF)
  _lcd.str24(60, 80, str(retry) + "/20", 0x07E0)

  if camera.handshake(retries=1, retry_ms=100):
    _log("Connected after %d retries" % retry, "CAM")
    break
  if camera.failed:
    _log("Camera self-test FAILED","CAM")
    break

gc.collect()

# =============================================================================
#                               Init: Tracker
# =============================================================================
_log("ObjectTracker...","INIT")
from ObjectTracker import ObjectTracker
tracker = ObjectTracker(motors, imu, camera, cfg)
_log("ObjectTracker OK","INIT")

# =============================================================================
#                               Init: Menu
# =============================================================================
_log("Menu...","INIT")
from Menu import MenuInit
menu = MenuInit(W=320, H=200, imu=imu, hdg=hdg, tracker=tracker, camera=camera,
                _lcd=_lcd, _lcd_drv=_lcd_drv)
_log("Menu OK","INIT")

# =============================================================================
#                               Buttons
# =============================================================================
LED = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
UP    = Pin('C8',  Pin.IN, pull=Pin.PULL_UP_47K)
DOWN  = Pin('C9',  Pin.IN, pull=Pin.PULL_UP_47K)
ENTER = Pin('C14', Pin.IN, pull=Pin.PULL_UP_47K)
BACK  = Pin('C15', Pin.IN, pull=Pin.PULL_UP_47K)

# =============================================================================
#                               Ticker
# =============================================================================
_log("Ticker...","INIT")
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
_log("Ticker OK","INIT")

# =============================================================================
#                               Main Loop
# =============================================================================
sleep_ms(100)
LED.low()
_log("Main loop running","Main")
keylast = [1, 1, 1, 1]

while True:
  # ---- BACK Safety Override ----
  if not BACK.value() and keylast[3]:
    if tracker.state != 'IDLE':
      tracker.stop()
    menu.handle_input('BACK')

  # ---- Other Keys ----
  if not UP.value() and keylast[0]:
    menu.handle_input('UP')
  if not DOWN.value() and keylast[1]:
    menu.handle_input('DOWN')
  if not ENTER.value() and keylast[2]:
    menu.handle_input('ENTER')

  keylast = [UP.value(), DOWN.value(), ENTER.value(), BACK.value()]

  # ---- Camera Polling ----
  new_frame = False
  if camera.is_ready():
    new_frame = camera.update()

  # ---- Control Update ----
  if tracker.state != 'IDLE':
    tracker.update()
  else:
    hdg.update()

  # ---- Display ----
  menu.update_display()
  sleep_ms(20)
