"""
calibrate_tcs.py — TCS3472 黄线标定脚本

用法: 在 REPL 运行:
  from sensors.calibrate_tcs import tcs_calibrate
  tcs_calibrate()

输出: 每秒打印 R/G/B/C、归一化 rn/gn/bn、yellow 布尔。
"""
from machine import I2C, Pin
from time import sleep_ms
from sensors.tcs3472 import TCS3472


def tcs_calibrate(i2c_id=1, scl=None, sda=None):
  """
  默认 i2c_id=1 → LPI2C2 SCL=C19 SDA=C18（本车接线）。
  """
  print("=== TCS3472 黄线标定 ===")
  print("bus: I2C(%d)  SCL=C19 SDA=C18" % i2c_id)

  try:
    if scl is not None and sda is not None:
      from machine import SoftI2C
      i2c = SoftI2C(scl=Pin(scl), sda=Pin(sda), freq=100000)
    else:
      i2c = I2C(i2c_id, freq=100000)
  except Exception as e:
    print("I2C init failed:", e)
    return

  devs = i2c.scan()
  print("I2C devices:", [hex(d) for d in devs])
  if 0x29 not in devs:
    print("WARN: TCS3472 (0x29) not found on bus!")

  tcs = TCS3472(i2c)
  print("thresh: rn>=%.2f gn>=%.2f bn<=%.2f C>=%d" % (
    tcs.yellow_r_min, tcs.yellow_g_min, tcs.yellow_b_max, tcs.yellow_c_min))
  print("Put on BLUE / YELLOW. Ctrl+C to stop.\n")

  while True:
    tcs.debug_print()
    sleep_ms(1000)
