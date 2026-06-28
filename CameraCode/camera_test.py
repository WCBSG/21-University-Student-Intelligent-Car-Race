"""
camera_test.py — OpenART Plus 通信测试程序 (OpenART Plus 端)
================================================================
配合 RT1021 端的 test_camera.py 使用，测试智能车与 OpenART 的 UART 通信。

硬件连接:
  OpenART TX  → RT1021 RX (D21)
  OpenART RX  → RT1021 TX (D20)
  OpenART GND → RT1021 GND

测试模式:
  1. UART 自检 — 短接 TX/RX，自发自收验证硬件
  2. 帧协议测试 — 发送/接收带 CRC 校验的数据帧
  3. 全功能测试 — 模型检测 + UART 收发 + 画面显示
  4. MCU 联调测试 — 接收 MCU 命令，发送检测结果

协议帧: [0xAA] [CMD:1B] [LEN:1B] [DATA:N bytes] [CRC:1B]

CMD 定义:
  0x0F — 心跳请求/响应
  0x10 — 目标检测结果
  0x20 — 文本消息 (UTF-8)
  0xF0 — 回环测试
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
  'uart_id': 12,              # OpenART UART 编号
  'baudrate': 460800,         # 波特率
  'img_width': 320,           # QVGA
  'img_height': 240,
  'conf_min': 70,             # 最低置信度 (%)
  'send_interval_ms': 50,     # 发送间隔
  'model_path': '/sd/yolo3_iou_smartcar_final_with_post_processing.tflite',
  'class_names': ['netball', 'sandbag', 'bear'],  # 类别名
}

# ============================== UART 分块写入 ==============================
# OpenART MicroPython 固件的 UART 内部缓冲区约 63-64 字节。
# 单次 write() 超过此限制会被截断。分块写入 + 块间延迟绕开此限制。

UART_CHUNK = 60        # 每块 ≤ 60 字节（留余量）
UART_CHUNK_DELAY = 2   # 块间延迟 ms（460800 baud 下 60 字节 ≈ 1.3ms）


def write_chunked(uart, data):
    """分块写入，绕过 UART 63 字节缓冲区限制"""
    for i in range(0, len(data), UART_CHUNK):
        chunk = data[i:i + UART_CHUNK]
        uart.write(chunk)
        if i + UART_CHUNK < len(data):
            time.sleep_ms(UART_CHUNK_DELAY)


# ============================== 协议工具 ==============================

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
  """发送一帧数据 [0xAA][CMD][LEN][DATA][CRC]"""
  if uart is None:
    return False
  escaped = escape(payload)
  length = len(escaped)
  if length > 255:
    print("  ! 帧过长:", length)
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


def recv_frame(uart, timeout_ms=10):
  """非阻塞接收一帧，返回 (cmd, payload) 或 (None, None)

  使用增量读取策略：UART RX 缓冲区只有 ~63 字节，无法一次性
  wait for N>63 字节，必须逐次读取累积。"""
  if uart is None:
    return None, None

  t0 = time.ticks_ms()

  # 找帧头 0xAA
  while uart.any() < 1:
    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
      return None, None
  sync = uart.read(1)
  if sync is None or sync[0] != 0xAA:
    return None, None

  # 读 CMD + LEN（增量读取，处理分块到达）
  t0 = time.ticks_ms()
  header = bytearray()
  while len(header) < 2:
    if uart.any():
      header.extend(uart.read(uart.any()))
    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
      return None, None
  cmd = header[0]
  length = header[1]

  # 读 payload + CRC（增量读取，绕过 RX 63 字节缓冲区限制）
  need = length + 1
  t0 = time.ticks_ms()
  data = bytearray()
  while len(data) < need:
    if uart.any():
      data.extend(uart.read(uart.any()))
    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
      return None, None

  payload = data[:length]
  crc_recv = data[length]

  # CRC 校验
  crc_calc = calc_crc(bytearray([cmd, length]) + payload)
  if crc_recv != crc_calc:
    print("  ! CRC 不匹配: calc={:02X} recv={:02X}".format(crc_calc, crc_recv))
    return None, None

  return cmd, unescape(payload)


# ============================== 检测结果编解码 ==============================

def encode_detections(objects):
  """
  编码检测结果为协议 payload
  objects: [(class_id, score, x1, y1, x2, y2), ...]
  payload: [num:1B] [cls:1B score:1B x1:1B y1:1B x2:1B y2:1B] ...
  """
  payload = bytearray()
  payload.append(len(objects))
  for cls_id, score, x1, y1, x2, y2 in objects:
    bx1 = max(0, min(255, int(x1 * 255)))
    by1 = max(0, min(255, int(y1 * 255)))
    bx2 = max(0, min(255, int(x2 * 255)))
    by2 = max(0, min(255, int(y2 * 255)))
    bscore = max(0, min(100, int(score * 100)))
    payload.extend(bytearray([cls_id, bscore, bx1, by1, bx2, by2]))
  return bytes(payload)


def decode_command(payload):
  """解码收到的命令 payload，返回 Python 基础类型"""
  try:
    return payload.decode('utf-8')
  except:
    return list(payload)


# ============================== 硬件自检 ==============================

def hw_check():
  """检查硬件是否就绪，返回 (ok, uart_obj, net_obj)"""
  print("\n" + "=" * 44)
  print("  [自检] OpenART Plus 硬件检查")
  print("=" * 44)

  all_ok = True

  # 1. 摄像头
  print("\n[1] 摄像头 (sensor)...")
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

  # 2. UART
  print("\n[2] UART (UART{})...".format(CFG['uart_id']))
  uart = None
  try:
    uart = UART(CFG['uart_id'], baudrate=CFG['baudrate'])
    print("  OK — {} baud".format(CFG['baudrate']))
  except Exception as e:
    print("  FAIL —", e)
    all_ok = False

  # 3. 模型
  print("\n[3] TF 模型...")
  net = None
  try:
    import tf
    net = tf.load(CFG['model_path'])
    print("  OK —", CFG['model_path'])
  except Exception as e:
    print("  SKIP (无模型，仅通信测试) —", e)

  # 总结
  status = "OK" if all_ok else "FAIL"
  print("\n---- 自检结果: {} ----".format(status))
  return all_ok, uart, net


# ============================== 测试 1: UART 回环自检 ==============================

def test_uart_loopback(uart):
  """
  UART 回环自检 (需要短接 OpenART 的 TX 和 RX)
  也可以在 OpenART 不接 MCU 时独立运行

  注意: OpenART MicroPython 固件的 UART 内部缓冲区约 63-64 字节。
  单次 write() 超过此限制会被固件截断（固件层限制，非硬件故障）。
  因此 ≤60 字节测试使用直写，>60 字节测试使用分块写入验证。
  """
  print("\n" + "=" * 44)
  print("  [测试1] UART 回环自检")
  print("=" * 44)
  print("  请短接 OpenART 的 TX 和 RX 引脚")
  print("  如果已连接 MCU，可跳过此测试")
  print("  注意: 固件 UART 缓冲区约 63 字节")
  print("        大包自动使用分块写入绕开限制")
  print("  3 秒后开始...")

  time.sleep_ms(3000)

  if uart is None:
    print("  SKIP — UART 未初始化")
    return False

  ok = 0
  fail = 0

  # 测试原始字节收发
  # ≤ 60 字节: 直写（不应截断）
  # > 60 字节: 分块写入（绕过 63 字节固件限制）
  test_sizes = [4, 16, 60, 64, 128, 255]
  for size in test_sizes:
    test_data = bytes([(j * 7 + 13) & 0xFF for j in range(size)])

    # 清缓冲
    while uart.any():
      uart.read(uart.any())

    if size <= UART_CHUNK:
      # 直写 — 验证固件缓冲区内不截断
      uart.write(test_data)
      time.sleep_ms(15)
      n = uart.any()
      if n >= size:
        rx = uart.read(n)
        if rx[:size] == test_data:
          print("  [{:>3d}B] OK".format(size))
          ok += 1
        else:
          print("  [{:>3d}B] FAIL — 数据不匹配".format(size))
          fail += 1
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
      # 最后再等一次
      time.sleep_ms(5)
      while uart.any():
        rx_all.extend(uart.read(uart.any()))

      if len(rx_all) >= size and bytes(rx_all[:size]) == test_data:
        print("  [{:>3d}B] (分块) OK".format(size))
        ok += 1
      else:
        print("  [{:>3d}B] (分块) FAIL — 收到 {} 字节".format(size, len(rx_all)))
        fail += 1

  # 测试帧协议 (使用 write_chunked 的 send_frame)
  print("\n  帧协议测试 (含转义, 分块写入)...")
  test_payload = bytes([0xAA, 0xBB, 0x00, 0x01, 0x55, 0xAA])
  while uart.any():
    uart.read(uart.any())

  send_frame(uart, 0xF0, test_payload)
  time.sleep_ms(20)

  cmd, rx_payload = recv_frame(uart, timeout_ms=30)
  if cmd == 0xF0 and rx_payload == test_payload:
    print("  帧协议 OK")
    ok += 1
  else:
    print("  帧协议 FAIL (需要短接TX-RX)")
    fail += 1

  total = ok + fail
  passed = (fail == 0)
  print("\n---- 回环测试: {}/{} OK ({}) ----".format(
    ok, total, "PASS" if passed else "FAIL"))

  if not passed:
    print("  提示: 如果 TX/RX 已连接到 MCU 而非短接，此测试应跳过")
    print("  提示: 固件 UART 缓冲区 ~63 字节，大包需分块写入")

  return passed


# ============================== 测试 2: 帧协议压力测试 ==============================

def test_frame_protocol(uart, iterations=20):
  """
  帧协议压力测试 — 发送多种 payload 尺寸和内容，
  验证 CRC 校验和转义机制。
  需要 MCU 端运行回环模式 (0xF0 命令原样返回)。

  MCU 端: test_camera.start()
  """
  print("\n" + "=" * 44)
  print("  [测试2] 帧协议压力测试")
  print("=" * 44)
  print("  需要 MCU 端运行 test_camera.start()")
  print("  确保 OpenART TX → MCU RX, OpenART RX → MCU TX")

  if uart is None:
    print("  SKIP — UART 未初始化")
    return False

  ok = 0
  fail = 0
  timeout = 0

  import random

  for i in range(iterations):
    # 随机 payload 长度 (0, 1, 10, 50, 200, 255)
    sizes = [0, 0, 1, 1, 10, 10, 50, 50, 200, 255]
    size = sizes[i % len(sizes)]

    # 随机 payload (包含转义字符)
    payload = bytes([random.randint(0, 255) for _ in range(size)])
    # 确保有些 payload 含 0xAA/0xBB
    if i % 3 == 0:
      payload = bytes([0xAA, 0xBB]) + payload[size // 3:]

    # 清缓冲
    while uart.any():
      uart.read(uart.any())

    send_frame(uart, 0xF0, payload)
    time.sleep_ms(30)

    cmd, rx = recv_frame(uart, timeout_ms=50)
    if cmd == 0xF0 and rx == payload:
      ok += 1
      if i % 10 == 0:
        print("  [#{:>2d}] {}B OK".format(i, size))
    elif cmd is None:
      timeout += 1
      print("  [#{:>2d}] {}B TIMEOUT".format(i, size))
      fail += 1
    else:
      fail += 1
      print("  [#{:>2d}] {}B FAIL — cmd=0x{:02X} rx={}".format(
        i, size, cmd, list(rx) if rx else []))

    time.sleep_ms(5)

  total = ok + fail
  passed = (fail == 0)
  print("\n---- 压力测试: OK={} TIMEOUT={} FAIL={} → {} ----".format(
    ok, timeout, fail - timeout, "PASS" if passed else "FAIL"))

  return passed


# ============================== 测试 3: 全功能测试 (检测 + 通信) ==============================

def test_full(uart, net, duration_sec=30):
  """
  全功能测试:
    - 运行目标检测模型
    - 在图像上绘制检测框
    - 将检测结果发送给 MCU
    - 接收并响应 MCU 命令
    - 统计 FPS 和通信成功率
  """
  print("\n" + "=" * 44)
  print("  [测试3] 全功能测试 ({} 秒)".format(duration_sec))
  print("=" * 44)

  if uart is None:
    print("  SKIP — UART 未初始化")
    return

  clock = time.clock()
  frame_count = 0
  detect_count = 0
  recv_count = 0
  send_ok = 0
  send_fail = 0
  last_send_ms = 0
  last_status_ms = time.ticks_ms()
  start_ms = time.ticks_ms()

  # 颜色表 (RGB565)
  COLORS = [0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF]

  print("  按 BACK 键或等待 {} 秒自动结束\n".format(duration_sec))

  while True:
    clock.tick()
    img = sensor.snapshot()
    frame_count += 1
    now = time.ticks_ms()

    # ---- 接收 MCU 命令 ----
    cmd, payload = recv_frame(uart, timeout_ms=5)
    if cmd is not None:
      recv_count += 1
      if cmd == 0xF0:       # 回环 — 原样返回
        send_frame(uart, 0xF0, payload)
      elif cmd == 0x0F:     # 心跳 — 回复
        send_frame(uart, 0x0F)
      elif cmd == 0x20:     # 文本消息
        text = decode_command(payload)
        print("  [MCU→] {}".format(text))

    # ---- 目标检测 + 发送 ----
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
          print("  ! 检测错误:", e)

      if objects:
        detect_count += 1

        # 绘制
        for cls_id, score, x1, y1, x2, y2 in objects:
          px1 = int(x1 * img.width())
          py1 = int(y1 * img.height())
          px2 = int(x2 * img.width())
          py2 = int(y2 * img.height())
          w = max(1, px2 - px1)
          h = max(1, py2 - py1)
          px1 = max(0, min(img.width() - 1, px1))
          py1 = max(0, min(img.height() - 1, py1))

          color = COLORS[cls_id % len(COLORS)]
          img.draw_rectangle((px1, py1, w, h), color=color, thickness=2)

          # 标签
          name = CFG['class_names'][cls_id] if cls_id < len(CFG['class_names']) else str(cls_id)
          lbl = "{} {:.0f}%".format(name, score * 100)
          img.draw_string(px1 + 2, max(0, py1 - 14), lbl, color=0xFFFF, scale=1)

        # 发送
        payload = encode_detections(objects)
        if send_frame(uart, 0x10, payload):
          send_ok += 1
        else:
          send_fail += 1

    # ---- 状态输出 ----
    if time.ticks_diff(now, last_status_ms) >= 3000:
      last_status_ms = now
      fps = clock.fps()
      elapsed = time.ticks_diff(now, start_ms) / 1000.0
      msg = "FPS:{:.1f} 检测:{} 收:{} 发:{}".format(
        fps, detect_count, recv_count, send_ok)
      print("  [{}s] {}".format(int(elapsed), msg))

    # ---- 结束 ----
    if time.ticks_diff(now, start_ms) >= duration_sec * 1000:
      break

    gc.collect()

  elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
  print("\n---- 全功能测试结束 ----")
  print("  时长: {:.1f}s".format(elapsed))
  print("  总帧: {}  检测帧: {}  收: {}  发: {}".format(
    frame_count, detect_count, recv_count, send_ok))
  if send_ok + send_fail > 0:
    print("  发送成功率: {:.1f}%".format(
      send_ok / (send_ok + send_fail) * 100))
  print("  平均 FPS: {:.1f}".format(frame_count / elapsed))


# ============================== 测试 4: MCU 命令响应测试 ==============================

def test_mcu_command(uart, duration_sec=15):
  """
  纯通信测试 (不运行模型):
    - 等待 MCU 发送各种命令
    - 统计收帧率和响应时间
    - 打印收到的文本消息
  适合调试通信协议，不需要模型文件。
  """
  print("\n" + "=" * 44)
  print("  [测试4] MCU 命令响应测试 ({} 秒)".format(duration_sec))
  print("=" * 44)
  print("  等待 MCU 命令... (无需模型)")

  if uart is None:
    print("  SKIP — UART 未初始化")
    return

  recv_count = 0
  cmd_stats = {}
  start_ms = time.ticks_ms()
  last_status_ms = start_ms

  while True:
    now = time.ticks_ms()

    cmd, payload = recv_frame(uart, timeout_ms=10)
    if cmd is not None:
      recv_count += 1
      cmd_stats[cmd] = cmd_stats.get(cmd, 0) + 1

      if cmd == 0xF0:
        # 回环 — 原样返回
        send_frame(uart, 0xF0, payload)
        print("  [回环] {}B -> 已回复".format(len(payload)))
      elif cmd == 0x0F:
        # 心跳
        send_frame(uart, 0x0F)
      elif cmd == 0x20:
        # 文本消息
        text = decode_command(payload)
        print("  [MCU] {}".format(text))
      elif cmd == 0x10:
        # MCU 发来了检测结果 (不常见，但兼容)
        print("  [检测数据] {}B".format(len(payload)))
      else:
        print("  [CMD=0x{:02X}] {}B".format(cmd, len(payload)))

    # 定时状态
    if time.ticks_diff(now, last_status_ms) >= 5000:
      last_status_ms = now
      elapsed = time.ticks_diff(now, start_ms) / 1000.0
      print("  [{:.0f}s] 总收帧: {}  {}".format(
        elapsed, recv_count,
        " ".join(["CMD_0x{:02X}:{}".format(k, v) for k, v in cmd_stats.items()])))

    if time.ticks_diff(now, start_ms) >= duration_sec * 1000:
      break

    time.sleep_ms(10)
    gc.collect()

  elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
  print("\n---- MCU 命令测试结束 ----")
  print("  时长: {:.1f}s  总收帧: {}".format(elapsed, recv_count))
  for k, v in sorted(cmd_stats.items()):
    print("    CMD 0x{:02X}: {} 帧".format(k, v))

  # 诊断
  if recv_count == 0:
    print("\n  *** 诊断: 未收到任何帧 ***")
    print("  1. 检查 TX/RX/GND 接线")
    print("  2. MCU 端是否运行 test_camera.start()")
    print("  3. 波特率是否一致 ({})".format(CFG['baudrate']))
  elif recv_count < 3:
    print("\n  *** 诊断: 收帧很少 ***")
    print("  可能存在 CRC 校验失败或接线不良")
    print("  尝试运行 test_camera.loopback() 排查 MCU 端问题")


# ============================== 入口 ==============================

def run(test_number=0, duration=30):
  """
  运行通信测试

  test_number:
    0 — 全部测试 (推荐)
    1 — 仅 UART 回环自检 (需短接 TX/RX)
    2 — 仅帧协议压力测试 (需 MCU)
    3 — 仅全功能测试 (需模型 + MCU)
    4 — 仅 MCU 命令响应测试 (需 MCU，无需模型)

  duration: 测试3/4 的时长 (秒)
  """
  print("\n" + "█" * 44)
  print("█  OpenART Plus ↔ 智能车 通信测试")
  print("█  时间: 启动就绪")
  print("█" * 44)

  # 自检
  all_ok, uart, net = hw_check()

  if test_number in (0, 1):
    test_uart_loopback(uart)

  if test_number in (0, 2):
    test_frame_protocol(uart, iterations=30)

  if test_number in (0, 3) and net is not None:
    test_full(uart, net, duration_sec=duration)

  if test_number in (0, 4):
    test_mcu_command(uart, duration_sec=duration)

  print("\n" + "█" * 44)
  print("█  通信测试全部完成")
  print("█" * 44)


# 当文件直接运行时执行全部测试
if __name__ == '__main__':
  run(test_number=3, duration=60)
