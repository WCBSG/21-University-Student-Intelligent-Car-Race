"""
CameraReceiver.py — OpenART Plus 检测结果 UART 接收器

协议帧: [0xAA] [CMD:1B] [LEN:1B] [DATA:N bytes] [CRC:1B]

CMD:
  0x01 — MCU→OpenART: Connection Request
  0x02 — OpenART→MCU: Self-Test Result (payload="200"/"400")
  0x03 — MCU→OpenART: Start Detection
  0x10 — OpenART→MCU: Detection Results

Detection payload (CMD 0x10):
  [num:1B] [cls_score:1B x:1B y:1B w:1B h:1B] × N
  cls_score = (cls:3bit << 5) | score:5bit
  x,y,w,h: 0-255 normalized (÷2.55 = %)

用法:
  camera = CameraReceiver(uart)
  camera.set_ready()  # 握手成功后调用
  if camera.update(): dets = camera.get_detections()
"""

import time

# ============================== 常量 ==============================

TIMEOUT_MS = 50
UART_CHUNK = 60
UART_CHUNK_DELAY = 2


# ============================== 协议工具 ==============================

def calc_crc(data):
  crc = 0
  for b in data:
    crc ^= b
  return crc & 0xFF


def escape(data):
  result = bytearray()
  for b in data:
    if b == 0xAA:
      result.extend(b'\xBB\x00')
    elif b == 0xBB:
      result.extend(b'\xBB\x01')
    else:
      result.append(b)
  return bytes(result)


def unescape(data):
  result = bytearray()
  i = 0
  while i < len(data):
    b = data[i]
    if b == 0xBB and i + 1 < len(data):
      if data[i + 1] == 0x00:
        result.append(0xAA)
        i += 2
        continue
      elif data[i + 1] == 0x01:
        result.append(0xBB)
        i += 2
        continue
    result.append(b)
    i += 1
  return bytes(result)


def write_chunked(uart, data):
  for i in range(0, len(data), UART_CHUNK):
    chunk = data[i:i + UART_CHUNK]
    uart.write(chunk)
    if i + UART_CHUNK < len(data):
      time.sleep_ms(UART_CHUNK_DELAY)


def send_frame(uart, cmd, payload=b''):
  escaped = escape(payload)
  length = len(escaped)
  header = bytearray([0xAA, cmd, length])
  crc = calc_crc(bytearray([cmd, length]) + escaped)
  frame = header + escaped + bytearray([crc])
  write_chunked(uart, frame)


def recv_frame(uart, timeout_ms=TIMEOUT_MS):
  n = uart.any()
  if n < 4:
    return None, None

  start = time.ticks_ms()
  while True:
    if uart.any() > 0:
      b = uart.read(1)
      if b and b[0] == 0xAA:
        break
    if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
      return None, None

  # 累积读取: header(2B) + payload(LEN) + CRC(1B) → buf
  buf = bytearray()
  start = time.ticks_ms()

  while len(buf) < 2:
    if uart.any():
      buf.extend(uart.read(uart.any()))
    if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
      return None, None

  cmd = buf[0]
  length = buf[1]

  need_total = 2 + length + 1
  while len(buf) < need_total:
    if uart.any():
      buf.extend(uart.read(uart.any()))
    if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
      return None, None

  payload = buf[2:2 + length]
  crc_recv = buf[2 + length]

  crc_calc = calc_crc(bytearray([cmd, length]) + payload)
  if crc_recv != crc_calc:
    return None, None

  return cmd, unescape(payload)


def parse_detections(payload):
  """
  新格式: [num:1B] [cls_score:1B x:1B y:1B w:1B h:1B] × N

  返回: [(cls_id, score_0_31, x%, y%, w%, h%, cx%, cy%, area, y2%), ...]
  """
  if len(payload) < 1:
    return []
  n = payload[0]
  OBJ_SIZE = 5
  if len(payload) < 1 + n * OBJ_SIZE:
    return []
  results = []
  for i in range(n):
    off = 1 + i * OBJ_SIZE
    cls_score = payload[off]
    cls_id = cls_score >> 5        # 3bit, 0-7
    score  = cls_score & 0x1F      # 5bit, 0-31
    x  = payload[off + 1] / 2.55   # → 0-100%
    y  = payload[off + 2] / 2.55
    w  = payload[off + 3] / 2.55
    h  = payload[off + 4] / 2.55
    cx = x + w / 2.0
    cy = y + h / 2.0
    y2 = y + h
    area = w * h                   # 0-10000
    results.append((cls_id, score, x, y, w, h, cx, cy, area, y2))
  return results


# ============================== CameraReceiver ==============================

class CameraReceiver:
  """
  OpenART Plus 检测结果接收器。

  握手由 main.py 完成。CameraReceiver 仅负责收帧+解析。
  """

  def __init__(self, uart):
    self._uart = uart
    self._detections = []
    self._ready = False
    self._failed = False   # 400 自检失败

  def set_ready(self):
    self._ready = True

  def is_ready(self):
    return self._ready

  @property
  def failed(self):
    """自检失败 (status=400)"""
    return self._failed

  def handshake(self, retries=50, retry_ms=100):
    """
    执行握手: 发 CMD 0x01 → 等 CMD 0x02 → 发 CMD 0x03。
    retries: 重试次数
    retry_ms: 每次重试间隔 (ms)
    返回: True=成功, False=超时/失败
    """
    recv_timeout = retry_ms - 20
    if recv_timeout < 10:
      recv_timeout = 10

    self._ready = False
    self._failed = False

    for _ in range(retries):
      send_frame(self._uart, 0x01)
      t0 = time.ticks_ms()
      while time.ticks_diff(time.ticks_ms(), t0) < recv_timeout:
        cmd, payload = recv_frame(self._uart, timeout_ms=10)
        if cmd == 0x02:
          status = payload.decode('utf-8') if payload else ""
          if status == "200":
            send_frame(self._uart, 0x03)
            time.sleep_ms(10)
            self._ready = True
            return True
          elif status == "400":
            self._failed = True
            return False
        time.sleep_ms(5)

    return False

  def update(self):
    """
    非阻塞轮询。收到 CMD 0x10 时解析缓存。
    返回: True = 本次调用收到了新检测帧
    """
    cmd, payload = recv_frame(self._uart)
    if cmd == 0x10:
      self._detections = parse_detections(payload)
      return True
    return False

  def get_detections(self):
    """
    返回最新检测结果:
      [(cls_id, score_0_31, x%, y%, w%, h%, cx%, cy%, area, y2%), ...]
    """
    return self._detections

  def flush(self):
    while self._uart.any():
      self._uart.read(self._uart.any())
    self._detections = []
