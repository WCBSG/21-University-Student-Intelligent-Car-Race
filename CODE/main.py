"""
-
-
- 请忽略  用于保护颈椎
-
-
"""

from machine import Pin
LED = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
# =============================================================================
#                               运动控制系统
# =============================================================================
from Motor import MotionControl

motors = MotionControl()


# =============================================================================
#                                陀螺仪系统
# =============================================================================
from imu import ImuSensor

# 陀螺仪 (IMU660RX + Madgwick 融合)
imu = ImuSensor(calibrate_samples=100, beta=0.05)


# =============================================================================
#                             航向角闭环控制
# =============================================================================
from HeadingController import HeadingController
from config import config as cfg

hdg = HeadingController(motors, imu, cfg)


# =============================================================================
#                                菜单系统
# =============================================================================
from Menu import MenuInit

menu = MenuInit(W=320, H=200, imu=imu, hdg=hdg)

# 按键
UP  = Pin('C8',  Pin.IN, pull=Pin.PULL_UP_47K)
DOWN  = Pin('C9',  Pin.IN, pull=Pin.PULL_UP_47K)
ENTER = Pin('C14', Pin.IN, pull=Pin.PULL_UP_47K)
BACK = Pin('C15', Pin.IN, pull=Pin.PULL_UP_47K)


# =============================================================================
#                                  定时器
# =============================================================================
from smartcar import ticker
import gc

tickCount = 0

def onTick(_):
  global tickCount
  tickCount += 1
  # 每帧运行 IMU 姿态融合
  imu.update()
  if tickCount >= 100:
    tickCount = 0
    gc.collect()

tkr = ticker(1)
tkr.capture_list(imu.raw)
tkr.callback(onTick)
tkr.start(10)


# =============================================================================
#                                  主程序
# =============================================================================
from time import sleep_ms
sleep_ms(100)

LED.low() # 初始化完成后开灯提示
keylast=[1,1,1,1]

while True:
  if not UP.value() and keylast[0]:
    menu.handle_input('UP')
  if not DOWN.value() and keylast[1]:
    menu.handle_input('DOWN')
  if not ENTER.value() and keylast[2]:
    menu.handle_input('ENTER')
  if not BACK.value() and keylast[3]:
    menu.handle_input('BACK')

  keylast = [UP.value(),DOWN.value(),ENTER.value(),BACK.value()]

  # 航向闭环控制 (IMU 数据由 ticker 自动采集+融合)
  hdg.update()

  menu.update_display()
  sleep_ms(20)
