"""
test_camera.py — RT1021 与 OpenART Plus UART 通信测试
----------------------------------------------------------
硬件连接:
  回环测试: D20(TX) — D21(RX) 杜邦线短接
  通信测试: D20(TX) → OpenART RX
            D21(RX) → OpenART TX
            GND     → OpenART GND

UART: LPUART6 (id=5) → TX=D20, RX=D21

协议帧: [0xAA] [CMD:1B] [LEN:1B] [DATA:N bytes] [CRC:1B]
  CMD 0x0F — 心跳
  CMD 0x10 — 目标检测结果
  CMD 0x20 — 文本消息
  CMD 0xF0 — 回环测试

用法:
  import test_camera
  test_camera.loopback()   # 回环测试 (先验证 UART 硬件)
  test_camera.start()      # OpenART 通信测试
"""

import gc, time
from machine import Pin, UART

# ============================== 常量 ==============================

UART_ID = 5          # LPUART6: TX=D20, RX=D21
BAUDRATE = 460800
LED_PIN = 'C4'
BACK_PIN = 'C15'
PING_INTERVAL_MS = 1000
TIMEOUT_FRAME_MS = 200

# ============================== UART 分块写入 ==============================
# RT1021 MicroPython 固件的 UART 同样存在 ~63-64 字节内部缓冲区限制。
# 分块写入 + 块间延迟绕开此限制。

UART_CHUNK = 60        # 每块 ≤ 60 字节
UART_CHUNK_DELAY = 2   # 块间延迟 ms


def write_chunked(uart, data):
    """分块写入，绕过 UART 63 字节缓冲区限制"""
    for i in range(0, len(data), UART_CHUNK):
        chunk = data[i:i + UART_CHUNK]
        uart.write(chunk)
        if i + UART_CHUNK < len(data):
            time.sleep_ms(UART_CHUNK_DELAY)


# ============================== 协议工具函数 ==============================

def calc_crc(data):
  """XOR 校验"""
  crc = 0
  for b in data:
    crc ^= b
  return crc & 0xFF


def escape(data):
  """转义: 0xAA→0xBB 0x00, 0xBB→0xBB 0x01"""
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
  """反转义: 0xBB 0x00→0xAA, 0xBB 0x01→0xBB"""
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


def send_frame(uart, cmd, payload=b''):
  """发送一帧（分块写入，绕过 UART 缓冲区限制）"""
  escaped = escape(payload)
  length = len(escaped)
  header = bytearray([0xAA, cmd, length])
  crc = calc_crc(bytearray([cmd, length]) + escaped)
  frame = header + escaped + bytearray([crc])
  write_chunked(uart, frame)


def recv_frame(uart):
  """非阻塞接收一帧。返回 (cmd, payload) 或 (None, None)

  使用增量读取策略：UART RX 缓冲区只有 ~63 字节，无法一次性
  wait for N>63 字节，必须逐次读取累积。"""
  n = uart.any()
  if n < 4:
    return None, None

  # 找帧头 0xAA
  start = time.ticks_ms()
  while True:
    if uart.any() > 0:
      b = uart.read(1)
      if b and b[0] == 0xAA:
        break
    if time.ticks_diff(time.ticks_ms(), start) > TIMEOUT_FRAME_MS:
      return None, None

  # 等 CMD + LEN（增量读取）
  start = time.ticks_ms()
  header = bytearray()
  while len(header) < 2:
    if uart.any():
      header.extend(uart.read(uart.any()))
    if time.ticks_diff(time.ticks_ms(), start) > TIMEOUT_FRAME_MS:
      return None, None

  cmd = header[0]
  length = header[1]

  # 等 payload + crc（增量读取，绕过 RX 63 字节缓冲区限制）
  need = length + 1
  start = time.ticks_ms()
  data = bytearray()
  while len(data) < need:
    if uart.any():
      data.extend(uart.read(uart.any()))
    if time.ticks_diff(time.ticks_ms(), start) > TIMEOUT_FRAME_MS:
      return None, None

  payload = data[:length]
  crc_recv = data[length]

  # CRC
  crc_calc = calc_crc(bytearray([cmd, length]) + payload)
  if crc_recv != crc_calc:
    return None, None

  return cmd, unescape(payload)


# ============================== 数据解析 ==============================

def parse_detections(payload):
  """
  解析检测结果
  格式: [num:1B] [obj_0:6B] ...
  obj_i = [class:1B] [score:1B] [x1:1B] [y1:1B] [x2:1B] [y2:1B]
  """
  if len(payload) < 1:
    return []
  n = payload[0]
  OBJ_SIZE = 6
  if len(payload) < 1 + n * OBJ_SIZE:
    return []
  results = []
  for i in range(n):
    off = 1 + i * OBJ_SIZE
    cls_id = payload[off]
    score = payload[off + 1]        # 0-100
    x1 = payload[off + 2] / 2.55    # → 0-100%
    y1 = payload[off + 3] / 2.55
    x2 = payload[off + 4] / 2.55
    y2 = payload[off + 5] / 2.55
    results.append((cls_id, score, x1, y1, x2, y2))
  return results


def print_detections(objects):
  """打印检测结果"""
  if not objects:
    return
  print("--- {} 个目标 ---".format(len(objects)))
  for i, (cls_id, score, x1, y1, x2, y2) in enumerate(objects):
    w = x2 - x1
    h = y2 - y1
    cx = x1 + w / 2
    cy = y1 + h / 2
    print("  [{}.{}] cls={:<3d} score={:>3d}% "
          "中心=({:5.1f}%,{:5.1f}%) 尺寸={:.1f}%x{:.1f}%".format(
          i, cls_id, cls_id, score, cx, cy, w, h))


def print_ascii_objects(objects, frame_w=32, frame_h=16):
  """ASCII 俯瞰图"""
  if not objects:
    return
  canvas = [['.' for _ in range(frame_w)] for _ in range(frame_h)]
  for cls_id, score, x1, y1, x2, y2 in objects:
    cx = int((x1 + x2) / 2.0 / 100.0 * frame_w)
    cy = int((y1 + y2) / 2.0 / 100.0 * frame_h)
    cx = max(0, min(frame_w - 1, cx))
    cy = max(0, min(frame_h - 1, cy))
    marker = str(cls_id % 10) if cls_id < 10 else '*'
    canvas[cy][cx] = marker
  print("  +" + "-" * frame_w + "+")
  for row in canvas:
    print("  |" + "".join(row) + "|")
  print("  +" + "-" * frame_w + "+")


# ============================== 回环测试 ==============================

def loopback(baudrate=None):
  """
  UART 回环测试 — 用杜邦线短接 D20(TX) 和 D21(RX)
  验证 UART 硬件是否正常。测试通过后再连接 OpenART。

  运行: test_camera.loopback()
  """
  br = baudrate if baudrate else BAUDRATE
  print("\n========== UART 回环测试 ==========")
  print("UART{} TX=D20 RX=D21 @ {} baud".format(UART_ID, br))
  print("请用杜邦线短接 D20 和 D21，按 BACK(C15) 开始...")
  print("注意: 固件 UART 缓冲区 ~63-64 字节，大包自动分块")

  BACK = Pin(BACK_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
  LED = Pin(LED_PIN, Pin.OUT, value=1)

  while BACK.value() == 1:
    time.sleep_ms(10)

  print("开始测试...\n")

  try:
    uart = UART(UART_ID, baudrate=br)
  except Exception as e:
    print("UART init fault:", e)
    return

  test_sizes = [4, 16, 60, 64, 128, 255]
  ok = 0
  fail = 0

  for size in test_sizes:
    # 生成测试数据
    test_data = bytearray()
    for j in range(size):
      test_data.append((j * 7 + 13) & 0xFF)

    # 清空缓冲
    while uart.any():
      uart.read(uart.any())

    if size <= UART_CHUNK:
      # 直写 — 验证固件缓冲区内不截断
      uart.write(test_data)
      time.sleep_ms(10)
      n = uart.any()
      if n >= size:
        rx = uart.read(n)
        if rx[:size] == test_data:
          print("  [{:>3d}B] OK".format(size))
          LED.off(); time.sleep_ms(30); LED.on()
          ok += 1
        else:
          print("  [{:>3d}B] FAIL — 数据不匹配".format(size))
          print("    发送:", bytes(test_data))
          print("    收到:", bytes(rx[:size]))
          fail += 1
          LED.off(); time.sleep_ms(200); LED.on()
          time.sleep_ms(200)
          LED.off(); time.sleep_ms(200); LED.on()
      else:
        print("  [{:>3d}B] FAIL — 收到 {} 字节 (固件缓冲区 ~63B 限制)".format(size, n))
        if n > 0:
          uart.read(n)
        fail += 1
    else:
      # 分块写入 + 逐块读取 — 绕过固件缓冲区限制
      # 在回环模式下 TX→RX 直连，必须边写边读否则 RX 缓冲区溢出
      rx_all = bytearray()
      for i in range(0, size, UART_CHUNK):
        chunk = test_data[i:i + UART_CHUNK]
        uart.write(chunk)
        time.sleep_ms(3)
        while uart.any():
          rx_all.extend(uart.read(uart.any()))
      time.sleep_ms(5)
      while uart.any():
        rx_all.extend(uart.read(uart.any()))

      if len(rx_all) >= size and bytes(rx_all[:size]) == bytes(test_data):
        print("  [{:>3d}B] (分块) OK".format(size))
        LED.off(); time.sleep_ms(30); LED.on()
        ok += 1
      else:
        print("  [{:>3d}B] (分块) FAIL — 收到 {} 字节".format(size, len(rx_all)))
        fail += 1

  # 测试帧协议 — 发送带转义的数据
  print("\n帧协议测试...")
  test_payload = bytearray([0xAA, 0xBB, 0x00, 0x01, 0x02, 0x55, 0xAA])
  while uart.any():
    uart.read(uart.any())

  send_frame(uart, 0xF0, test_payload)
  time.sleep_ms(20)

  cmd, rx_payload = recv_frame(uart)
  if cmd == 0xF0 and rx_payload == bytes(test_payload):
    print("  帧协议 OK (payload={}B, 含转义字符)".format(len(rx_payload)))
    ok += 1
  else:
    print("  帧协议 FAIL")
    if cmd:
      print("    cmd=0x{:02X} payload={}".format(cmd, rx_payload))
    fail += 1

  total = ok + fail
  print("\n========== 结果: {}/{} OK ==========".format(ok, total))

  if fail == 0:
    print("全部通过! 请拔掉回环线，连接 OpenART Plus。")
    print("然后运行: test_camera.start()")
  else:
    print("存在失败项，请检查:")
    print("  1. 杜邦线是否短接 D20 和 D21")
    print("  2. 引脚是否被其他设备占用")
    print("  3. 波特率是否匹配")
    print("  注意: RT1021 固件 UART 缓冲区 ~63-64 字节")
    print("  大包需通过分块写入或帧协议发送，直写会被截断")


# ============================== OpenART 通信测试 ==============================

def start(baudrate=None):
  """
  连接到 OpenART Plus 并开始通信测试。
  确保 OpenART 端已上电并运行 CameraCode/main.py。

  运行: test_camera.start()
  """
  br = baudrate if baudrate else BAUDRATE
  print("\n" + "=" * 50)
  print("RT1021 <-UART-> OpenART Plus 通信测试")
  print("TX=D20 RX=D21 @ {} baud".format(br))
  print("=" * 50)

  LED = Pin(LED_PIN, Pin.OUT, value=1)
  BACK = Pin(BACK_PIN, Pin.IN, pull=Pin.PULL_UP_47K)

  try:
    uart = UART(UART_ID, baudrate=br)
    print("UART init OK\n")
  except Exception as e:
    print("UART init fault:", e)
    return

  last_ping = time.ticks_ms()
  frame_count = 0
  detect_count = 0
  last_status = time.ticks_ms()

  print("等待 OpenART 数据... (LED 闪 = 收帧, BACK = 退出)\n")

  while True:
    # ---- 接收 ----
    cmd, payload = recv_frame(uart)

    if cmd is not None:
      frame_count += 1
      LED.off()
      time.sleep_ms(5)
      LED.on()

      if cmd == 0x10:  # 检测结果
        detect_count += 1
        objects = parse_detections(payload)
        if objects:
          print("\n[检测#{:<4d}] {} 个目标:".format(detect_count, len(objects)))
          print_detections(objects)
          print_ascii_objects(objects)

      elif cmd == 0x20:  # 文本
        try:
          msg = payload.decode('utf-8')
          print("[OpenART]", msg)
        except:
          print("[OpenART RAW]", payload)

      elif cmd == 0x0F:  # 心跳响应
        pass

      elif cmd == 0xF0:  # 回环
        print("[回环] {} 字节匹配".format(len(payload)))

      else:
        print("[CMD=0x{:02X}] len={}".format(cmd, len(payload)))

    # ---- 心跳 + 状态 ----
    now = time.ticks_ms()
    if time.ticks_diff(now, last_ping) >= PING_INTERVAL_MS:
      last_ping = now
      send_frame(uart, 0x0F)

    # 每 5 秒打印状态
    if time.ticks_diff(now, last_status) >= 5000:
      last_status = now
      if frame_count > 0:
        print("[状态] 收帧:{} 检测帧:{}".format(frame_count, detect_count))

    # ---- 退出 ----
    if BACK.value() == 0:
      print("\n用户退出。")
      print("统计: 总帧={} 检测帧={}".format(frame_count, detect_count))
      break

    gc.collect()
    time.sleep_ms(5)


# ============================== 导入提示 ==============================

print("""
╔══════════════════════════════════════════╗
║  test_camera.py 已加载                   ║
║                                          ║
║  测试步骤:                                ║
║  1. test_camera.loopback() 回环测试      ║
║  2. test_camera.start()   OpenART 通信   ║
║                                          ║
║  连线:                                    ║
║  回环: D20 — D21 (杜邦线)               ║
║  通信: D20→OpenART RX                    ║
║        D21→OpenART TX                    ║
║        GND→OpenART GND                   ║
╚══════════════════════════════════════════╝
""")