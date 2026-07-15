"""
camera.py — UART 帧协议 + CameraRx 接收器（二合一）

帧: [0xAA][CMD][LEN][DATA:LEN][CRC]
转义: 0xAA→BB 00, 0xBB→BB 01
CMD: 0x01(连接请求) 0x02(自检结果) 0x03(开始检测) 0x10(检测帧)
"""

import time

TIMEOUT_MS = 50

# =============================================================================
#                         CRC / 转义 / UART 收发
# =============================================================================

def _crc(data):
  c = 0
  for b in data: c ^= b
  return c & 0xFF


def _escape(data):
  r = bytearray()
  for b in data:
    if b == 0xAA: r.extend(b'\xBB\x00')
    elif b == 0xBB: r.extend(b'\xBB\x01')
    else: r.append(b)
  return bytes(r)


def _unescape(data):
  r = bytearray(); i = 0
  while i < len(data):
    b = data[i]
    if b == 0xBB and i + 1 < len(data):
      if data[i + 1] == 0x00: r.append(0xAA); i += 2; continue
      if data[i + 1] == 0x01: r.append(0xBB); i += 2; continue
    r.append(b); i += 1
  return bytes(r)


def _send_frame(uart, cmd, payload=b''):
  esc = _escape(payload)
  if len(esc) > 255: return False
  hdr = bytearray([0xAA, cmd, len(esc)])
  crc = _crc(bytearray([cmd, len(esc)]) + esc)
  try:
    for i in range(0, len(hdr) + len(esc) + 1, 60):
      uart.write((hdr + esc + bytearray([crc]))[i:i + 60])
      if i + 60 < len(hdr) + len(esc) + 1: time.sleep_ms(2)
    return True
  except Exception: return False


def _recv_frame(uart, timeout_ms=TIMEOUT_MS):
  if uart.any() < 4: return None, None
  t0 = time.ticks_ms()
  while True:
    if uart.any() > 0:
      b = uart.read(1)
      if b and b[0] == 0xAA: break
    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms: return None, None
  buf = bytearray(); t0 = time.ticks_ms()
  while len(buf) < 2:
    if uart.any(): buf.extend(uart.read(min(uart.any(), 2 - len(buf))))
    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms: return None, None
  cmd, length = buf[0], buf[1]
  need = 2 + length + 1
  while len(buf) < need:
    if uart.any(): buf.extend(uart.read(min(uart.any(), need - len(buf))))
    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms: return None, None
  payload = buf[2:2 + length]
  if buf[2 + length] != _crc(bytearray([cmd, length]) + payload): return None, None
  return cmd, _unescape(payload)


def _recv_handshake(uart, timeout_ms=80):
  t0 = time.ticks_ms()
  while True:
    if uart.any() > 0:
      b = uart.read(1)
      if b and b[0] == 0xAA: break
    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms: return None, None
  buf = bytearray(); t0 = time.ticks_ms()
  while len(buf) < 2:
    if uart.any(): buf.extend(uart.read(min(uart.any(), 2 - len(buf))))
    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms: return None, None
  cmd, length = buf[0], buf[1]
  need = 2 + length + 1
  while len(buf) < need:
    if uart.any(): buf.extend(uart.read(min(uart.any(), need - len(buf))))
    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms: return None, None
  payload = buf[2:2 + length]
  if buf[2 + length] != _crc(bytearray([cmd, length]) + payload): return None, None
  return cmd, _unescape(payload)


def _parse(payload):
  if len(payload) < 1: return []
  n = payload[0]
  if len(payload) < 1 + n * 5: return []
  r = []
  for i in range(n):
    off = 1 + i * 5
    cs = payload[off]
    r.append((
      cs >> 5, cs & 0x1F,
      payload[off + 1] / 2.55, payload[off + 2] / 2.55,
      payload[off + 3] / 2.55, payload[off + 4] / 2.55,
    ))
  # add derived fields: cx, cy, area, y2
  return [(*d, d[2] + d[4] / 2, d[3] + d[5] / 2, d[4] * d[5], d[3] + d[5]) for d in r]


# =============================================================================
#                         CameraRx — 非阻塞接收器
# =============================================================================

class DetectionFrame:
  def __init__(self, num, detections):
    self.num = num; self.detections = detections
    self.has_target = num > 0


class CameraRx:
  def __init__(self, uart, timeout_ms=5000):
    self._uart = uart; self._timeout_ms = timeout_ms
    self._dets = []; self._last_ms = 0; self._ready = False; self._failed = False
    self._lost = 0

  def poll(self):
    result = None
    while True:
      cmd, payload = _recv_frame(self._uart)
      if cmd == 0x10:
        self._dets = _parse(payload) if payload else []
        self._last_ms = time.ticks_ms(); self._lost = 0
        result = DetectionFrame(len(self._dets), self._dets)
      elif cmd is None: break
    if result is None: self._lost += 1
    return result

  def handshake(self, retries=50, retry_ms=100):
    rt = max(10, retry_ms - 20)
    self._ready = self._failed = False; self.flush()
    for _ in range(retries):
      _send_frame(self._uart, 0x01)
      t0 = time.ticks_ms()
      while time.ticks_diff(time.ticks_ms(), t0) < rt:
        cmd, payload = _recv_handshake(self._uart, timeout_ms=rt)
        if cmd == 0x02:
          if (payload or b"").decode('utf-8') == "200":
            _send_frame(self._uart, 0x03); time.sleep_ms(10)
            self._ready = True; self._last_ms = time.ticks_ms()
            return True
          self._failed = True; return False
        time.sleep_ms(5)
    return False

  @property
  def detections(self): return self._dets
  @property
  def has_target(self): return len(self._dets) > 0
  @property
  def timed_out(self):
    return self._last_ms and time.ticks_diff(time.ticks_ms(), self._last_ms) > self._timeout_ms
  @property
  def is_ready(self): return self._ready
  @property
  def failed(self): return self._failed
  @property
  def lost_count(self): return self._lost

  def set_ready(self):
    self._ready = True; self._last_ms = time.ticks_ms()

  def flush(self):
    for _ in range(256):
      if not self._uart.any(): break
      self._uart.read(self._uart.any())
    self._dets = []
