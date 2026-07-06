"""
CameraCode/main.py — OpenART Plus 正式检测程序

上电 → 自检(sensor+uart+model) → 等待MCU连接请求 → 握手 → 检测循环

Protocol:
  CMD 0x01 — MCU→OpenART: Connection Request
  CMD 0x02 — OpenART→MCU: Self-Test Result (payload="200"/"400")
  CMD 0x03 — MCU→OpenART: Start Detection
  CMD 0x10 — OpenART→MCU: Detection Results

Detection payload (CMD 0x10):
  [num:1B] [cls_score:1B x:1B y:1B w:1B h:1B] × N
  cls_score = (cls:3bit << 5) | score:5bit (0-31)
"""

import sensor, image, time, gc

try:
  from machine import UART
except ImportError:
  from pyb import UART

try:
  import tf
except ImportError:
  tf = None

# ============================== 配置 ==============================

CFG = {
  'uart_id': 12,
  'baudrate': 460800,
  'img_width': 320,
  'img_height': 240,
  'conf_min': 70,              # 最低置信度 (%)
  'send_interval_ms': 50,
  'model_path': '/sd/yolo3_iou_smartcar_final_with_post_processing.tflite',
  'class_names': ['netball', 'sandbag', 'bear'],
}

# ============================== 协议工具 ==============================

UART_CHUNK = 60
UART_CHUNK_DELAY = 2


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
  if length > 255:
    print("  ! Frame too long:", length)
    return False
  header = bytearray([0xAA, cmd, length])
  crc = calc_crc(bytearray([cmd, length]) + escaped)
  frame = header + escaped + bytearray([crc])
  try:
    write_chunked(uart, frame)
    return True
  except Exception as e:
    print("  ! UART write error:", e)
    return False


def recv_frame(uart, timeout_ms=100):
  """非阻塞接收一帧。累积缓冲区防止贪婪读。"""
  if uart is None:
    return None, None

  t0 = time.ticks_ms()

  while uart.any() < 1:
    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
      return None, None
  sync = uart.read(1)
  if sync is None or sync[0] != 0xAA:
    return None, None

  # 累积读取: 全部字节进 buf，再提取 header+payload+CRC
  buf = bytearray()
  t0 = time.ticks_ms()

  while len(buf) < 2:
    if uart.any():
      buf.extend(uart.read(uart.any()))
    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
      return None, None

  cmd = buf[0]
  length = buf[1]

  need_total = 2 + length + 1
  while len(buf) < need_total:
    if uart.any():
      buf.extend(uart.read(uart.any()))
    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
      return None, None

  payload = buf[2:2 + length]
  crc_recv = buf[2 + length]
  crc_calc = calc_crc(bytearray([cmd, length]) + payload)
  if crc_recv != crc_calc:
    return None, None

  return cmd, unescape(payload)


# ============================== 新编码格式 ==============================

def encode_detections(objects):
  """
  objects: [(cls_id, score, x1, y1, x2, y2), ...]
  新格式: [num:1B] [cls_score:1B x:1B y:1B w:1B h:1B] × N
  cls_score = (cls:3bit << 5) | score:5bit
  """
  payload = bytearray()
  payload.append(len(objects))
  for cls_id, score, x1, y1, x2, y2 in objects:
    c = max(0, min(7, int(cls_id)))
    s5 = max(0, min(31, int(score * 31)))
    payload.append((c << 5) | s5)
    # x, y, w, h: 归一化 0-255
    payload.append(max(0, min(255, int(x1 * 255))))
    payload.append(max(0, min(255, int(y1 * 255))))
    payload.append(max(0, min(255, int((x2 - x1) * 255))))
    payload.append(max(0, min(255, int((y2 - y1) * 255))))
  return bytes(payload)


# ============================== 自检 ==============================

def self_test():
  """自检: sensor + UART + model。返回 (success, uart, net)"""
  print("\n" + "=" * 40)
  print("  OpenART Self-Test")
  print("=" * 40)

  all_ok = True

  # Sensor
  print("\n[1] Sensor...")
  try:
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=500)
    img = sensor.snapshot()
    print("  OK — {}x{}".format(img.width(), img.height()))
  except Exception as e:
    print("  FAIL —", e)
    all_ok = False

  # UART
  print("\n[2] UART (UART{})...".format(CFG['uart_id']))
  uart = None
  try:
    uart = UART(CFG['uart_id'], baudrate=CFG['baudrate'])
    print("  OK — {} baud".format(CFG['baudrate']))
  except Exception as e:
    print("  FAIL —", e)
    all_ok = False

  # Model
  print("\n[3] Model...")
  net = None
  try:
    import tf
    net = tf.load(CFG['model_path'])
    print("  OK —", CFG['model_path'])
  except Exception as e:
    print("  FAIL —", e)
    all_ok = False

  status = "OK" if all_ok else "FAIL"
  print("\n---- Self-Test: {} ----".format(status))
  return all_ok, uart, net


# ============================== 主程序 ==============================

def main():
  print("\n" + "=" * 40)
  print("  OpenART Plus Detection Program")
  print("=" * 40)

  # 1. Self-test
  ok, uart, net = self_test()
  status = b"200" if ok else b"400"

  if uart is None:
    print("FATAL: UART not available, halted")
    return

  # 2. Wait for MCU connection request (CMD 0x01)
  print("\nWaiting for MCU connection (CMD 0x01)...")
  while True:
    cmd, payload = recv_frame(uart, timeout_ms=100)
    if cmd == 0x01:
      print("  Received CMD 0x01")
      break
    time.sleep_ms(10)
    gc.collect()

  # 3. Send self-test result (CMD 0x02)
  send_frame(uart, 0x02, status)
  print("  Sent CMD 0x02: " + status.decode('utf-8'))

  if status == b"400":
    print("  Self-test failed. Halting.")
    return

  # 4. Wait for start command (CMD 0x03)
  print("  Waiting for start (CMD 0x03)...")
  while True:
    cmd, payload = recv_frame(uart, timeout_ms=100)
    if cmd == 0x01:             # MCU 重发了连接请求 → 重新回复 0x02
      send_frame(uart, 0x02, status)
    elif cmd == 0x03:
      print("  Received CMD 0x03 — starting detection")
      break
    time.sleep_ms(10)
    gc.collect()

  # 5. Detection loop
  clock = time.clock()
  frame_count = 0
  detect_count = 0
  send_ok = 0
  last_send_ms = 0
  last_status_ms = time.ticks_ms()
  start_ms = time.ticks_ms()

  print("  Detection loop running...\n")

  while True:
    clock.tick()
    img = sensor.snapshot()
    frame_count += 1
    now = time.ticks_ms()

    # Detection + send
    if time.ticks_diff(now, last_send_ms) >= CFG['send_interval_ms']:
      last_send_ms = now

      objects = []
      if net is not None:
        try:
          for obj in tf.detect(net, img):
            x1, y1, x2, y2, label, score = obj
            if score > CFG['conf_min'] / 100.0:
              objects.append((
                int(label), float(score),
                float(x1), float(y1), float(x2), float(y2)
              ))
        except Exception as e:
          pass  # 单帧检测失败静默跳过

      if objects:
        detect_count += 1
        payload = encode_detections(objects)
        if send_frame(uart, 0x10, payload):
          send_ok += 1

    # Periodic status
    if time.ticks_diff(now, last_status_ms) >= 5000:
      last_status_ms = now
      fps = clock.fps()
      elapsed = time.ticks_diff(now, start_ms) / 1000.0
      print("  [{}s] FPS:{:.1f} 检测:{} 发:{}".format(
        int(elapsed), fps, detect_count, send_ok))

    gc.collect()


# ============================== 入口 ==============================

if __name__ == '__main__':
  main()
