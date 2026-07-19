from machine import Pin
import time
from time import ticks_ms, ticks_diff
from log import info
class BitBangI2C:
  def __init__(self, scl_pin, sda_pin, freq=100000):
    self._sc = Pin(scl_pin, Pin.OPEN_DRAIN, pull=Pin.PULL_UP_47K)
    self._sd = Pin(sda_pin, Pin.OPEN_DRAIN, pull=Pin.PULL_UP_47K)
    self._us = max(1, 500000 // freq)
    self._sc(1); self._sd(1)
    time.sleep_us(10)
  def _start(self):
    self._sd(1); time.sleep_us(self._us)
    self._sc(1); time.sleep_us(self._us)
    self._sd(0); time.sleep_us(self._us)
    self._sc(0); time.sleep_us(self._us)
  def _stop(self):
    self._sd(0); time.sleep_us(self._us)
    self._sc(1); time.sleep_us(self._us)
    self._sd(1); time.sleep_us(self._us)
  def _wb(self, b):
    for _ in range(8):
      self._sd(1 if b & 0x80 else 0); time.sleep_us(self._us)
      self._sc(1); time.sleep_us(self._us)
      self._sc(0)
      b = (b << 1) & 0xFF
    self._sd(1); time.sleep_us(self._us)
    self._sc(1); time.sleep_us(self._us)
    ack = self._sd.value() == 0
    self._sc(0); time.sleep_us(self._us)
    return ack
  def _rb(self, ack=True):
    b = 0
    self._sd(1)
    for _ in range(8):
      time.sleep_us(self._us)
      self._sc(1); time.sleep_us(self._us)
      b = (b << 1) | self._sd.value()
      self._sc(0)
    self._sd(0 if ack else 1); time.sleep_us(self._us)
    self._sc(1); time.sleep_us(self._us)
    self._sc(0); time.sleep_us(self._us)
    self._sd(1)
    return b
  def writeto(self, addr, data, stop=True):
    self._start()
    if not self._wb(addr << 1):
      self._stop()
      raise OSError(5, "NACK 0x%02X" % addr)
    for b in data:
      self._wb(b)
    if stop:
      self._stop()
  def readfrom_mem(self, addr, reg, nbytes):
    self._start()
    if not self._wb(addr << 1):
      self._stop(); raise OSError(5, "NACK 0x%02X" % addr)
    if not self._wb(reg):
      self._stop(); raise OSError(5, "NACK reg 0x%02X" % reg)
    self._start()
    if not self._wb((addr << 1) | 1):
      self._stop(); raise OSError(5, "NACK read 0x%02X" % addr)
    buf = bytearray(nbytes)
    for i in range(nbytes):
      buf[i] = self._rb(ack=(i < nbytes - 1))
    self._stop()
    return bytes(buf)
def make_i2c(freq=100000):
  return BitBangI2C('C19', 'C18', freq=freq)
class TCS3472:
  def __init__(self, i2c, addr=0x29, gain=0x00, atime=0xC0):
    self._i2c = i2c
    self._addr = addr
    self._gain = gain
    self._atime = atime
    self._inited = False
    self.confirm_n = 2
    self._on_line = False
    self._y_streak = 0
    self._n_streak = 0
    self._err_n = 0
    self._last_err_ms = 0
    self._last_rgb = (0, 0, 0, 0)
    self._read_ok = False
    self._reinit_count = 0
    self._reinit_next_ms = 0
    self._gave_up = False
    self._wr_buf = bytearray(2)
    self.yellow_r_min = 0.28
    self.yellow_g_min = 0.28
    self.yellow_b_max = 0.25
    self.yellow_c_min = 800
    self.init()
  def init(self):
    if self._gave_up:
      return
    try:
      id_val = self._read8(0x92)
      if id_val != 0x44 and id_val != 0x4D:
        info("TCS", "WARN: ID 0x%02X" % id_val)
      self._write8(0x80, 0x03)
      self._write8(0x81, self._atime)
      self._write8(0x8F, self._gain)
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
    if not self._inited:
      self._read_ok = False
      self._on_i2c_error()
      return self._last_rgb
    try:
      data = self._i2c.readfrom_mem(self._addr, 0x94, 8)
      c = (data[1] << 8) | data[0]
      r = (data[3] << 8) | data[2]
      g = (data[5] << 8) | data[4]
      b = (data[7] << 8) | data[6]
      self._last_rgb = (r, g, b, c)
      self._err_n = 0
      self._read_ok = True
      return self._last_rgb
    except OSError:
      self._read_ok = False
      self._on_i2c_error()
      return self._last_rgb
  def _classify_yellow(self, r, g, b, c):
    if not self._read_ok or c < self.yellow_c_min:
      return False
    rn = r / c
    gn = g / c
    bn = b / c
    return (rn >= self.yellow_r_min and
            gn >= self.yellow_g_min and
            bn <= self.yellow_b_max)
  def is_yellow(self):
    return self._classify_yellow(*self.read_raw())
  def sample(self):
    raw = self.is_yellow()
    rising = False
    if raw:
      self._n_streak = 0
      self._y_streak += 1
      if (not self._on_line) and self._y_streak >= self.confirm_n:
        self._on_line = True
        rising = True
    else:
      self._y_streak = 0
      self._n_streak += 1
      if self._on_line and self._n_streak >= self.confirm_n:
        self._on_line = False
    return rising
  def crossed_yellow(self):
    return self.sample()
  def reset_crossed(self):
    self._y_streak = 0
    self._n_streak = 0
  @property
  def on_line(self):
    return self._on_line
  @property
  def read_ok(self):
    return self._read_ok
  def last_rgb(self):
    r, g, b, c = self._last_rgb
    if c > 0:
      rn, gn, bn = r / c, g / c, b / c
    else:
      rn = gn = bn = 0.0
    return (r, g, b, c, rn, gn, bn, self._read_ok)
  def _on_i2c_error(self):
    if self._gave_up:
      return
    self._err_n += 1
    now = ticks_ms()
    if ticks_diff(now, self._last_err_ms) > 2000:
      self._last_err_ms = now
      info("TCS", "I2C EIO x%d (skip frame)" % self._err_n)
    if (self._err_n >= 8 and not self._gave_up
        and ticks_diff(now, self._reinit_next_ms) >= 0):
      self._err_n = 0
      backoff_ms = 1000 * (1 << min(self._reinit_count, 4))
      self._reinit_next_ms = now + backoff_ms
      if self._reinit_count >= 5:
        self._gave_up = True
        info("TCS", "gave up after %d re-init failures" % self._reinit_count)
        return
      info("TCS", "re-init attempt %d (backoff %dms)..." % (self._reinit_count + 1, backoff_ms))
      self.init()
  def _write8(self, reg, val):
    buf = self._wr_buf
    buf[0] = reg
    buf[1] = val
    self._i2c.writeto(self._addr, buf)
  def _read8(self, reg):
    return self._i2c.readfrom_mem(self._addr, reg, 1)[0]
  def _read16(self, reg):
    data = self._i2c.readfrom_mem(self._addr, reg, 2)
    return (data[1] << 8) | data[0]
