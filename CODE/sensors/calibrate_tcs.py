"""
calibrate_tcs.py — TCS3472 黄线标定脚本

用法: 在 main.py 的 init 阶段调用 tcs_calibrate(i2c_id)
      或单独在 REPL 中运行。

输出: 每秒打印一次 R/G/B/C + yellow 布尔值。
      人工分别放在蓝布和黄胶带上，记录两边的 R/G/B 典型值，
      填入 tcs3472.py 的 yellow_r_min / yellow_g_min / yellow_b_max。
"""
from machine import I2C, Pin
from time import sleep_ms
from sensors.tcs3472 import TCS3472


def tcs_calibrate(i2c_id=0, scl=None, sda=None):
  """
  i2c_id: I2C 模块编号 (0-3), 见 E07 demo 引脚表
  scl/sda: 若提供则用 SoftI2C 或指定引脚
  """
  print("=== TCS3472 黄线标定 ===")

  try:
    if scl is not None and sda is not None:
      from machine import SoftI2C
      i2c = SoftI2C(scl=Pin(scl), sda=Pin(sda), freq=100000)
    else:
      i2c = I2C(i2c_id, freq=100000)
  except Exception as e:
    print("I2C init failed:", e)
    return

  # 扫描设备
  devs = i2c.scan()
  print("I2C devices:", [hex(d) for d in devs])
  if 0x29 not in devs:
    print("WARN: TCS3472 (0x29) not found on bus!")

  tcs = TCS3472(i2c)
  print("Put sensor on BLUE background...")
  sleep_ms(2000)
  print("Now on YELLOW tape...")
  sleep_ms(2000)
  print("Starting continuous read (1Hz). Ctrl+C to stop.\n")

  while True:
    tcs.debug_print()
    sleep_ms(1000)
