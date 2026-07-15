# CameraCode/main.py — OpenART Plus (自包含)
import sensor, image, time, gc
try:
  from machine import UART
except ImportError:
  from pyb import UART
try:
  import tf
except ImportError:
  tf = None

# 0=sandbag/左 1=netball/上 2=bear/右（与 MCU config 一致）
CLASS_NAMES = ('sandbag', 'netball', 'bear')
UART_ID = 12
BAUD = 460800
CONF_MIN = 0.70
SEND_MS = 50
MAX_OBJ = 5
MODEL = '/sd/yolo3_iou_smartcar_final_with_post_processing.tflite'
_CHUNK = 60
_CHUNK_MS = 2


def _label_to_cls(label):
  if isinstance(label, bool):
    return None
  if isinstance(label, int):
    if 0 <= label < len(CLASS_NAMES):
      return label
    return None
  if isinstance(label, float):
    i = int(label)
    if abs(label - i) < 1e-3 and 0 <= i < len(CLASS_NAMES):
      return i
    return None
  try:
    return CLASS_NAMES.index(str(label))
  except (ValueError, TypeError):
    return None


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
      if data[i + 1] == 0x01:
        result.append(0xBB)
        i += 2
        continue
    result.append(b)
    i += 1
  return bytes(result)


def write_chunked(uart, data):
  for i in range(0, len(data), _CHUNK):
    uart.write(data[i:i + _CHUNK])
    if i + _CHUNK < len(data):
      time.sleep_ms(_CHUNK_MS)


def send_frame(uart, cmd, payload=b''):
  escaped = escape(payload)
  length = len(escaped)
  if length > 255:
    return False
  header = bytearray([0xAA, cmd, length])
  crc = calc_crc(bytearray([cmd, length]) + escaped)
  try:
    write_chunked(uart, header + escaped + bytearray([crc]))
    return True
  except Exception:
    return False


def recv_frame(uart, timeout_ms=100):
  if uart is None or uart.any() < 4:
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
      need = 2 - len(buf)
      buf.extend(uart.read(min(uart.any(), need)))
    if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
      return None, None
  cmd = buf[0]
  length = buf[1]
  need_total = 2 + length + 1
  while len(buf) < need_total:
    if uart.any():
      need = need_total - len(buf)
      buf.extend(uart.read(min(uart.any(), need)))
    if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
      return None, None
  payload = buf[2:2 + length]
  if buf[2 + length] != calc_crc(bytearray([cmd, length]) + payload):
    return None, None
  return cmd, unescape(payload)


def encode_detections(objects):
  n = len(objects)
  if n > 255:
    n = 255
  payload = bytearray()
  payload.append(n)
  for i in range(n):
    cls_id, score, x1, y1, x2, y2 = objects[i]
    c = max(0, min(7, int(cls_id)))
    s5 = max(0, min(31, int(score * 31 + 0.5)))
    payload.append((c << 5) | s5)
    payload.append(max(0, min(255, int(x1 * 255))))
    payload.append(max(0, min(255, int(y1 * 255))))
    w = x2 - x1
    h = y2 - y1
    if w < 0:
      w = 0
    if h < 0:
      h = 0
    payload.append(max(0, min(255, int(w * 255))))
    payload.append(max(0, min(255, int(h * 255))))
  return bytes(payload)


def self_test():
  print("[CAM] Self-test...")
  all_ok = True
  try:
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=500)
    img = sensor.snapshot()
    print("[CAM] Sensor OK {}x{}".format(img.width(), img.height()))
  except Exception as e:
    print("[CAM] Sensor FAIL:", e)
    all_ok = False
  uart = None
  try:
    uart = UART(UART_ID, baudrate=BAUD)
    print("[CAM] UART OK")
  except Exception as e:
    print("[CAM] UART FAIL:", e)
    all_ok = False
  net = None
  try:
    net = tf.load(MODEL)
    print("[CAM] Model OK")
  except Exception as e:
    print("[CAM] Model FAIL:", e)
    all_ok = False
  print("[CAM] Self-test: " + ("OK" if all_ok else "FAIL"))
  return all_ok, uart, net


def main():
  ok, uart, net = self_test()
  status = b"200" if ok else b"400"
  if uart is None:
    print("[CAM] FATAL: no UART")
    return
  if not ok:
    print("[CAM] Will report 400")
  while uart.any():
    uart.read(uart.any())

  while True:
    print("[CAM] Wait CMD 0x01...")
    while True:
      cmd, _ = recv_frame(uart, timeout_ms=100)
      if cmd == 0x01:
        break
      time.sleep_ms(10)
      gc.collect()

    send_frame(uart, 0x02, status)
    print("[CAM] Sent 0x02: " + status.decode('utf-8'))
    if status == b"400":
      continue

    while True:
      cmd, _ = recv_frame(uart, timeout_ms=100)
      if cmd == 0x01:
        send_frame(uart, 0x02, status)
      elif cmd == 0x03:
        print("[CAM] Detection start")
        break
      time.sleep_ms(10)
      gc.collect()

    last_send_ms = 0
    last_gc_ms = time.ticks_ms()

    while True:
      now = time.ticks_ms()
      cmd, _ = recv_frame(uart, timeout_ms=1)
      if cmd == 0x01:
        print("[CAM] Re-handshake")
        send_frame(uart, 0x02, status)
        t_rehs = time.ticks_ms()
        timed_out = False
        while not timed_out:
          cmd2, _ = recv_frame(uart, timeout_ms=100)
          if cmd2 == 0x01:
            send_frame(uart, 0x02, status)
          elif cmd2 == 0x03:
            print("[CAM] Re-handshake done")
            break
          if time.ticks_diff(time.ticks_ms(), t_rehs) > 5000:
            print("[CAM] Re-handshake timeout")
            timed_out = True
          time.sleep_ms(10)
        if timed_out:
          break
        while uart.any():
          uart.read(uart.any())
        last_send_ms = time.ticks_ms()
        last_gc_ms = last_send_ms
        continue

      if time.ticks_diff(now, last_send_ms) >= SEND_MS:
        img = sensor.snapshot()
        # ROI: 底部 25%（物体在地面，顶部无用），与 openart_mul_od 一致
        img1 = img.copy(0.75, 1)
        objects = []
        if net is not None:
          try:
            for obj in tf.detect(net, img1):
              x1, y1, x2, y2, label, score = obj
              score = float(score)
              if score <= CONF_MIN:
                continue
              cls_id = _label_to_cls(label)
              if cls_id is None:
                continue
              objects.append((cls_id, score, float(x1), float(y1), float(x2), float(y2)))
          except Exception as e:
            print("[CAM] tf err:", e)
            objects = []
        if len(objects) > MAX_OBJ:
          objects.sort(key=lambda o: -o[1])
          objects = objects[:MAX_OBJ]



        # 画面标注 — 调试用（不影响 UART 发送）
        try:
          W, H = img.width(), img.height()
          for cls_id, score, x1, y1, x2, y2 in objects:
            px = int(x1 * W)
            py = int(y1 * H)
            pw = int((x2 - x1) * W)
            ph = int((y2 - y1) * H)
            img.draw_rectangle((px, py, pw, ph), thickness=2)
            label = CLASS_NAMES[cls_id] + ' ' + str(int(score * 100)) + '%'
            img.draw_string(px + 2, max(0, py - 12), label)
        except Exception as e:
          print(e)



        if send_frame(uart, 0x10, encode_detections(objects)):
          last_send_ms = now

      if time.ticks_diff(now, last_gc_ms) >= 3000:
        last_gc_ms = now
        gc.collect()


main()
