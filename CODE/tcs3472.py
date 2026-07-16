"""
sensors/tcs3472.py — TCS3472XFN 颜色传感器驱动

AMS TCS3472XFN, I2C 7-bit addr 0x29, RGB + Clear.
车底贴地安装，检测黄线（蓝布背景 vs 黄胶带）。

I2C 偶发 EIO（震动/接触）不抛异常：本拍当作非黄，连续失败会尝试重初始化。
"""

from time import ticks_ms, ticks_diff
from log import info

# 本车接线: SCL=C19 SDA=C18（GPIO bit-bang，不依赖硬件 I2C）

_REG_ENABLE  = 0x80
_REG_ATIME   = 0x81
_REG_CONTROL = 0x8F
_REG_ID      = 0x92
_REG_CDATA   = 0x94
_REG_RDATA   = 0x96
_REG_GDATA   = 0x98
_REG_BDATA   = 0x9A

_INTEGRATION_154MS = 0xC0
_GAIN_1X = 0x00


def make_i2c(freq=100000):
  from bitbang_i2c import BitBangI2C
  return BitBangI2C('C19', 'C18', freq=freq)


class TCS3472:
  """TCS3472XFN。I2C 7-bit addr = 0x29。"""

  def __init__(self, i2c, addr=0x29, gain=_GAIN_1X, atime=_INTEGRATION_154MS):
    self._i2c = i2c
    self._addr = addr
    self._gain = gain
    self._atime = atime
    self._prev_yellow = False
    self._yellow_count = 0
    self._inited = False
    self.confirm_n = 2
    self._on_line = False
    self._y_streak = 0
    self._n_streak = 0
    self._err_n = 0
    self._last_err_ms = 0
    self._last_rgb = (0, 0, 0, 0)
    self._read_ok = False
    # 重试控制
    self._reinit_count = 0
    self._reinit_next_ms = 0
    self._gave_up = False

    # 实地标定: 蓝布 rn≈0.19 gn≈0.32 bn≈0.44; 黄胶 rn≈0.37 gn≈0.39 bn≈0.16
    self.yellow_r_min = 0.28
    self.yellow_g_min = 0.28
    self.yellow_b_max = 0.25
    self.yellow_c_min = 800

    self.init()

  def init(self):
    if self._gave_up:
      return
    try:
      id_val = self._read8(_REG_ID)
      if id_val != 0x44 and id_val != 0x4D:
        info("TCS", "WARN: ID 0x%02X" % id_val)
      self._write8(_REG_ENABLE, 0x03)
      self._write8(_REG_ATIME, self._atime)
      self._write8(_REG_CONTROL, self._gain)
      self._inited = True
      self._err_n = 0
      self._read_ok = False
      self._reinit_count = 0
    except OSError as e:
      self._inited = False
      self._read_ok = False
      self._reinit_count += 1
      info("TCS", "init EIO: %s (attempt %d)" % (e, self._reinit_count))

  def read_raw(self):
    """返回 (R, G, B, C)。I2C 失败返回上次值或全 0，不抛异常。"""
    if not self._inited:
      self._read_ok = False
      self._on_i2c_error()
      return self._last_rgb
    try:
      c = self._read16(_REG_CDATA)
      r = self._read16(_REG_RDATA)
      g = self._read16(_REG_GDATA)
      b = self._read16(_REG_BDATA)
      self._last_rgb = (r, g, b, c)
      self._err_n = 0
      self._read_ok = True
      return self._last_rgb
    except OSError:
      self._read_ok = False
      self._on_i2c_error()
      return self._last_rgb

  def read_rgb(self):
    r, g, b, c = self.read_raw()
    if c < 10:
      return (0.0, 0.0, 0.0)
    return (r / c, g / c, b / c)

  def is_yellow(self):
    r, g, b, c = self.read_raw()
    # 旧读数只供诊断显示；绝不能在通信失败时继续生成黄线事件。
    if not self._read_ok:
      return False
    if c < self.yellow_c_min:
      return False
    rn = r / c
    gn = g / c
    bn = b / c
    return (rn >= self.yellow_r_min and
            gn >= self.yellow_g_min and
            bn <= self.yellow_b_max)

  def crossed_yellow(self):
    """
    黄线 OFF→ON 上升沿（滞回）。I2C 失败本拍视为非黄，不崩主循环。
    """
    try:
      raw = self.is_yellow()
    except OSError:
      self._on_i2c_error()
      raw = False
    rising = False
    if raw:
      self._n_streak = 0
      self._y_streak += 1
      if (not self._on_line) and self._y_streak >= self.confirm_n:
        self._on_line = True
        rising = True
        self._yellow_count += 1
    else:
      self._y_streak = 0
      self._n_streak += 1
      if self._on_line and self._n_streak >= self.confirm_n:
        self._on_line = False
    self._prev_yellow = raw
    return rising

  def reset_crossed(self):
    self._y_streak = 0
    self._n_streak = 0

  @property
  def on_line(self):
    return self._on_line

  @property
  def yellow_cross_count(self):
    return self._yellow_count

  def debug_print(self):
    r, g, b, c = self.read_raw()
    if c > 0:
      rn, gn, bn = r / c, g / c, b / c
    else:
      rn = gn = bn = 0.0
    is_y = (self._read_ok and c >= self.yellow_c_min and
            rn >= self.yellow_r_min and
            gn >= self.yellow_g_min and
            bn <= self.yellow_b_max)
    info("TCS", "R=%d G=%d B=%d C=%d | rn=%.2f gn=%.2f bn=%.2f yellow=%s" % (
      r, g, b, c, rn, gn, bn, is_y))
    return (r, g, b, c, is_y)

  def _on_i2c_error(self):
    if self._gave_up:
      return
    self._err_n += 1
    now = ticks_ms()
    if ticks_diff(now, self._last_err_ms) > 2000:
      self._last_err_ms = now
      info("TCS", "I2C EIO x%d (skip frame)" % self._err_n)
    # 连续失败 + 退避时间到 → 尝试重初始化
    if (self._err_n >= 8 and not self._gave_up
        and ticks_diff(now, self._reinit_next_ms) >= 0):
      self._err_n = 0
      # 指数退避: 1s, 2s, 4s, 8s, 16s
      backoff_ms = 1000 * (1 << min(self._reinit_count, 4))
      self._reinit_next_ms = now + backoff_ms
      if self._reinit_count >= 5:
        self._gave_up = True
        info("TCS", "gave up after %d re-init failures" % self._reinit_count)
        return
      info("TCS", "re-init attempt %d (backoff %dms)..." % (self._reinit_count + 1, backoff_ms))
      self.init()

  def _write8(self, reg, val):
    self._i2c.writeto(self._addr, bytearray([reg, val]))

  def _read8(self, reg):
    return self._i2c.readfrom_mem(self._addr, reg, 1)[0]

  def _read16(self, reg):
    data = self._i2c.readfrom_mem(self._addr, reg, 2)
    return (data[1] << 8) | data[0]
