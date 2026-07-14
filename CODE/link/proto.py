"""
proto.py — UART 帧协议（MCU 与 OpenART 共享）

部署: 脚本/手动拷贝此文件到 CameraCode/ 目录，禁止两边手写拷贝。

帧格式: [0xAA] [CMD:1B] [LEN:1B] [DATA:LEN] [CRC:1B]
转义: 0xAA→0xBB 0x00, 0xBB→0xBB 0x01

CMD:
  0x01 — MCU→OpenART: Connection Request
  0x02 — OpenART→MCU: Self-Test Result ("200"/"400")
  0x03 — MCU→OpenART: Start Detection
  0x10 — OpenART→MCU: Detection Results

Detection payload (CMD 0x10):
  [num:1B] [cls_score:1B x:1B y:1B w:1B h:1B] × N
  cls_score = (cls:3bit << 5) | score:5bit
"""

import time

# ============================== 常量 ==============================

TIMEOUT_MS = 50
UART_CHUNK = 60
UART_CHUNK_DELAY = 2
OBJ_SIZE = 5  # 每个检测目标的字节数


# ============================== CRC / 转义 ==============================

def calc_crc(data):
  """XOR 校验和，1 字节。"""
  crc = 0
  for b in data:
    crc ^= b
  return crc & 0xFF


def escape(data):
  """字节填充：0xAA→BB 00, 0xBB→BB 01。"""
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
  """字节去填充。"""
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


# ============================== UART 发送 ==============================

def write_chunked(uart, data):
  """分片写入 UART，防止缓冲区溢出。"""
  for i in range(0, len(data), UART_CHUNK):
    chunk = data[i:i + UART_CHUNK]
    uart.write(chunk)
    if i + UART_CHUNK < len(data):
      time.sleep_ms(UART_CHUNK_DELAY)


def send_frame(uart, cmd, payload=b''):
  """构造帧并发送。返回 True/False。长度 >255 时拒绝。"""
  escaped = escape(payload)
  length = len(escaped)
  if length > 255:
    return False
  header = bytearray([0xAA, cmd, length])
  crc = calc_crc(bytearray([cmd, length]) + escaped)
  frame = header + escaped + bytearray([crc])
  try:
    write_chunked(uart, frame)
    return True
  except Exception as e:
    return False


# ============================== UART 接收 ==============================

def recv_frame(uart, timeout_ms=TIMEOUT_MS):
  """
  非阻塞接收。要求 ≥4 字节才尝试（适配高频轮询）。
  逐字节搜索 0xAA，按需读取避免丢弃后续帧数据。
  返回 (cmd, payload) 或 (None, None)。
  """
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

  buf = bytearray()
  start = time.ticks_ms()

  while len(buf) < 2:
    if uart.any():
      needed = 2 - len(buf)
      buf.extend(uart.read(min(uart.any(), needed)))
    if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
      return None, None

  cmd = buf[0]
  length = buf[1]

  need_total = 2 + length + 1
  while len(buf) < need_total:
    if uart.any():
      needed = need_total - len(buf)
      buf.extend(uart.read(min(uart.any(), needed)))
    if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
      return None, None

  payload = buf[2:2 + length]
  crc_recv = buf[2 + length]

  crc_calc = calc_crc(bytearray([cmd, length]) + payload)
  if crc_recv != crc_calc:
    return None, None

  return cmd, unescape(payload)


def recv_frame_handshake(uart, timeout_ms=80):
  """
  握手专用接收：不要求最小字节数，阻塞等待完整帧。
  超时返回 (None, None)。逐字节搜索 + 按需读取。
  """
  t0 = time.ticks_ms()

  while True:
    if uart.any() > 0:
      b = uart.read(1)
      if b and b[0] == 0xAA:
        break
    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
      return None, None

  buf = bytearray()
  t0 = time.ticks_ms()

  while len(buf) < 2:
    if uart.any():
      needed = 2 - len(buf)
      buf.extend(uart.read(min(uart.any(), needed)))
    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
      return None, None

  cmd = buf[0]
  length = buf[1]

  need_total = 2 + length + 1
  while len(buf) < need_total:
    if uart.any():
      needed = need_total - len(buf)
      buf.extend(uart.read(min(uart.any(), needed)))
    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
      return None, None

  payload = buf[2:2 + length]
  crc_recv = buf[2 + length]

  crc_calc = calc_crc(bytearray([cmd, length]) + payload)
  if crc_recv != crc_calc:
    return None, None

  return cmd, unescape(payload)


# ============================== 检测编解码 ==============================

def encode_detections(objects):
  """
  objects: [(cls_id, score, x1, y1, x2, y2), ...]
  返回 bytes payload:
    [num:1B] [cls_score:1B x:1B y:1B w:1B h:1B] × N
    cls_score = (cls:3bit << 5) | score:5bit (四舍五入)
  """
  payload = bytearray()
  payload.append(len(objects))
  for cls_id, score, x1, y1, x2, y2 in objects:
    c = max(0, min(7, int(cls_id)))
    s5 = max(0, min(31, int(score * 31 + 0.5)))
    payload.append((c << 5) | s5)
    payload.append(max(0, min(255, int(x1 * 255))))
    payload.append(max(0, min(255, int(y1 * 255))))
    payload.append(max(0, min(255, int((x2 - x1) * 255))))
    payload.append(max(0, min(255, int((y2 - y1) * 255))))
  return bytes(payload)


def parse_detections(payload):
  """
  解析检测 payload。
  返回: [(cls_id, score_0_31, x%, y%, w%, h%, cx%, cy%, area, y2%), ...]
  """
  if len(payload) < 1:
    return []
  n = payload[0]
  if len(payload) < 1 + n * OBJ_SIZE:
    return []
  results = []
  for i in range(n):
    off = 1 + i * OBJ_SIZE
    cls_score = payload[off]
    cls_id = cls_score >> 5
    score = cls_score & 0x1F
    x = payload[off + 1] / 2.55
    y = payload[off + 2] / 2.55
    w = payload[off + 3] / 2.55
    h = payload[off + 4] / 2.55
    cx = x + w / 2.0
    cy = y + h / 2.0
    y2 = y + h
    area = w * h
    results.append((cls_id, score, x, y, w, h, cx, cy, area, y2))
  return results
