"""
boot_mode.py — 启动模式标志

优先级:
  1. /flash/boot_mode 文件（DEBUG 长按 C20 写入后 soft reset）— 读完即删
  2. 上电稳定按住 C20 ≥ hold_ms → MATCH（真赛：按住再上电）
  3. 默认 DEBUG

注意: C20 上电瞬间可能抖动，hold 必须足够长且连续为低，避免误进 MATCH。
"""

import os

BOOT_FILE = "/flash/boot_mode"


def read_and_clear():
  """读启动文件并删除。返回 "DEBUG" | "MATCH"。"""
  try:
    with open(BOOT_FILE, "r") as f:
      mode = f.read().strip()
  except OSError:
    return "DEBUG"
  try:
    os.remove(BOOT_FILE)
  except OSError:
    pass
  if mode in ("MATCH", "DEBUG"):
    return mode
  return "DEBUG"


def _c20_held_stable(c20_pin, hold_ms):
  """
  连续 hold_ms 内采样均为按下(0) 才算按住。
  任一次读到松开(1)立即失败。上电先等引脚稳定。
  """
  from time import sleep_ms, ticks_ms, ticks_diff

  sleep_ms(30)  # 上电电平稳定
  if c20_pin.value() != 0:
    return False

  t0 = ticks_ms()
  while ticks_diff(ticks_ms(), t0) < hold_ms:
    if c20_pin.value() != 0:
      return False
    sleep_ms(10)
  # 再确认一次
  return c20_pin.value() == 0


def resolve_boot_mode(c20_pin, hold_ms=1000):
  """
  综合判定启动模式。
  先处理文件（软重启进 MATCH），再检测上电长按 C20。
  默认 hold_ms=1000，减少误触发。
  """
  # 1) 文件（调试态长按写入后的 soft reset）
  file_mode = read_and_clear()
  if file_mode == "MATCH":
    print("[BOOT] reason=file → MATCH")
    return "MATCH"

  # 2) 上电按住 C20（真赛）
  if _c20_held_stable(c20_pin, hold_ms):
    print("[BOOT] reason=C20_hold_%dms → MATCH" % hold_ms)
    return "MATCH"

  print("[BOOT] reason=default C20=%d → DEBUG" % c20_pin.value())
  return "DEBUG"


def clear_boot_file():
  """强制删除 boot 文件（调试用）。"""
  try:
    os.remove(BOOT_FILE)
    print("[BOOT] cleared %s" % BOOT_FILE)
  except OSError:
    pass


def request_reboot(mode="MATCH"):
  """写 flag 并软复位。"""
  ok = False
  try:
    with open(BOOT_FILE, "w") as f:
      f.write(mode)
    ok = True
  except OSError as e:
    print("[BOOT] write %s failed: %s" % (BOOT_FILE, e))
  print("[BOOT] reset → %s (file_ok=%s)" % (mode, ok))
  import machine
  machine.reset()
