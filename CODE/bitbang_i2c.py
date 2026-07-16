from machine import Pin
import time


class BitBangI2C:
  """GPIO bit-bang I2C，仅提供 writeto/readfrom_mem。"""

  def __init__(self, scl_pin, sda_pin, freq=100000):
    self._sc = Pin(scl_pin, Pin.OPEN_DRAIN)
    self._sd = Pin(sda_pin, Pin.OPEN_DRAIN)
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
