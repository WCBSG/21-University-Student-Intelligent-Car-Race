"""
sensors/tcs3472.py — TCS3472XFN 颜色传感器驱动

AMS TCS3472XFN, I2C 7-bit addr 0x29, RGB + Clear.
车底贴地安装，检测黄线（蓝布背景 vs 黄胶带）。

用法:
  from machine import I2C
  i2c = I2C(0, freq=100000)          # 或 I2C(1/2/3), 查 E07 demo 引脚表
  tcs = TCS3472(i2c)
  r, g, b, c = tcs.read_raw()
  if tcs.crossed_yellow():
      print("越过黄线!")
"""

# TCS3472 寄存器 (CMD bit 7 = 1, auto-increment)
_REG_ENABLE  = 0x80  # 0x00: PON(bit0) + AEN(bit1)
_REG_ATIME   = 0x81  # 0x01: 积分时间
_REG_CONTROL = 0x8F  # 0x0F: 增益
_REG_ID      = 0x92  # 0x12: 器件 ID (应为 0x44)
_REG_STATUS  = 0x93  # 0x13: bit0 = AVALID
_REG_CDATA   = 0x94  # 0x14: Clear (2B)
_REG_RDATA   = 0x96  # 0x16: Red   (2B)
_REG_GDATA   = 0x98  # 0x18: Green (2B)
_REG_BDATA   = 0x9A  # 0x1A: Blue  (2B)

# 积分时间 (ATIME)
# T = 2.4ms * (256 - ATIME), 默认 ATIME=0xC0 → T=2.4*64=153.6ms
_INTEGRATION_154MS = 0xC0   # 153.6ms, 推荐
_INTEGRATION_700MS = 0x00   # 614.4ms, 弱光

# 增益
_GAIN_1X  = 0x00
_GAIN_4X  = 0x01
_GAIN_16X = 0x10
_GAIN_60X = 0x11


class TCS3472:
  """TCS3472XFN 颜色传感器。I2C 7-bit addr = 0x29。"""

  def __init__(self, i2c, addr=0x29, gain=_GAIN_1X, atime=_INTEGRATION_154MS):
    self._i2c = i2c
    self._addr = addr
    self._gain = gain
    self._atime = atime
    self._prev_yellow = False
    self._yellow_count = 0
    self._inited = False

    # 可调黄色阈值 — 实地标定后修改
    self.yellow_r_min = 120   # R 通道最小值 (0-65535 raw)
    self.yellow_g_min = 100   # G 通道最小值
    self.yellow_b_max = 80    # B 通道最大值 (黄色 B 低)

    self.init()

  # ————————————————————————————————————————————————————————
  #                      初始化
  # ————————————————————————————————————————————————————————

  def init(self):
    """上电并配置传感器。"""
    id_val = self._read8(_REG_ID)
    if id_val != 0x44 and id_val != 0x4D:
      print("[TCS] WARN: unexpected ID 0x%02X (expected 0x44/0x4D)" % id_val)

    # 上电 + 使能 RGBC
    self._write8(_REG_ENABLE, 0x03)  # PON | AEN
    # 积分时间
    self._write8(_REG_ATIME, self._atime)
    # 增益
    self._write8(_REG_CONTROL, self._gain)
    self._inited = True

  # ————————————————————————————————————————————————————————
  #                      原始读取
  # ————————————————————————————————————————————————————————

  def read_raw(self):
    """返回 (R, G, B, C) — 16-bit 原始值。"""
    if not self._inited:
      return (0, 0, 0, 0)

    c = self._read16(_REG_CDATA)
    r = self._read16(_REG_RDATA)
    g = self._read16(_REG_GDATA)
    b = self._read16(_REG_BDATA)
    return (r, g, b, c)

  def read_rgb(self):
    """返回 (R, G, B) 归一化 0.0-1.0（基于 Clear 归一化）。"""
    r, g, b, c = self.read_raw()
    if c < 10:  # 太暗，防除零
      return (0.0, 0.0, 0.0)
    return (r / c, g / c, b / c)

  # ————————————————————————————————————————————————————————
  #                      黄线检测
  # ————————————————————————————————————————————————————————

  def is_yellow(self):
    """
    当前读数是否为黄色（黄胶带）。
    判据: R > threshold AND G > threshold AND B < threshold (黄 = 高R+高G+低B)
    阈值需实地标定。蓝布上: R≈40 G≈60 B≈100; 黄胶带上: R≈200 G≈180 B≈50 (示例)
    """
    r, g, b, c = self.read_raw()
    if c < 20:
      return False  # 太暗，无判据
    # 归一化到 ~0-255 便于设阈值 (除以 c * 255)
    # 也可以用绝对值，看标定结果
    return (r > self.yellow_r_min and
            g > self.yellow_g_min and
            b < self.yellow_b_max)

  def crossed_yellow(self):
    """
    黄线上升沿检测: 上次不是黄、这次是黄 → True。
    每拍调用一次，用于 PUSH 阶段检测「推出黄线」瞬间。
    """
    now = self.is_yellow()
    crossed = (now and not self._prev_yellow)
    self._prev_yellow = now
    if crossed:
      self._yellow_count += 1
    return crossed

  def reset_crossed(self):
    """复位上升沿计数（每次 SCORE 后调用）。"""
    self._prev_yellow = False

  @property
  def yellow_cross_count(self):
    return self._yellow_count

  # ————————————————————————————————————————————————————————
  #                      Debug 打印
  # ————————————————————————————————————————————————————————

  def debug_print(self):
    """打印当前 raw 值 + 黄线判据（P0 标定用）。"""
    r, g, b, c = self.read_raw()
    is_y = self.is_yellow()
    print("[TCS] R=%d G=%d B=%d C=%d yellow=%s" % (r, g, b, c, is_y))
    return (r, g, b, c, is_y)

  # ————————————————————————————————————————————————————————
  #                      I2C 底层
  # ————————————————————————————————————————————————————————

  def _write8(self, reg, val):
    self._i2c.writeto(self._addr, bytearray([reg, val]))

  def _read8(self, reg):
    return self._i2c.readfrom_mem(self._addr, reg, 1)[0]

  def _read16(self, reg):
    data = self._i2c.readfrom_mem(self._addr, reg, 2)
    return (data[1] << 8) | data[0]  # LSB first
