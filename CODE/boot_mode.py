"""
boot_mode.py — 启动模式标志（文件法）

C20 按 → request_match_reboot("MATCH") → machine.reset()
上电 → read_and_clear() → "DEBUG" | "MATCH"
"""

import os

BOOT_FILE = "/flash/boot_mode"


def read_and_clear():
  """启动时调用。返回 "DEBUG" 或 "MATCH"，读完即清文件。"""
  try:
    with open(BOOT_FILE, "r") as f:
      mode = f.read().strip()
  except OSError:
    return "DEBUG"
  # 读完即清，防止下次上电误进 MATCH
  try:
    os.remove(BOOT_FILE)
  except OSError:
    pass
  return mode if mode in ("MATCH", "DEBUG") else "DEBUG"


def request_reboot(mode="MATCH"):
  """写 flag 并软复位。"""
  try:
    with open(BOOT_FILE, "w") as f:
      f.write(mode)
  except OSError:
    pass
  import machine
  machine.reset()
