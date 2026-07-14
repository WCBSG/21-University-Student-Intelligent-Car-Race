"""
camera_rx.py — Camera 接收器（排空/空帧/超时）

用法:
  cam = CameraRx(uart, timeout_ms=5000)
  # 每拍:
  frame = cam.poll()          # → DetectionFrame | None
  if cam.timed_out: ...       # → FAULT
"""

import time
from link.proto import recv_frame, recv_frame_handshake, send_frame, parse_detections


class DetectionFrame:
  """一帧检测结果。"""

  def __init__(self, num, detections, seq=None):
    self.num = num                 # 目标数量（含 0=空帧）
    self.detections = detections   # [(cls_id, score, x%, y%, w%, h%, cx%, cy%, area, y2%), ...]
    self.has_target = num > 0
    self.seq = seq                 # 可选帧序号

  def __repr__(self):
    return "DetectionFrame(num=%d, has=%s)" % (self.num, self.has_target)


class CameraRx:
  """
  非阻塞 Camera 接收器。

  poll() 排空当前 UART 缓冲中的所有帧，返回最后一帧。
  空帧 (num=0) 清空内部缓存，让 has_target 及时变为 False。
  timed_out 属性用于 FSM 检测链路断开。
  """

  def __init__(self, uart, timeout_ms=5000):
    self._uart = uart
    self._timeout_ms = timeout_ms
    self._detections = []
    self._last_frame_ms = 0
    self._ready = False
    self._failed = False
    self._lost_count = 0   # 连续无帧计数（可选统计）

  # ——————————————————————————————————————————————————————————
  #                      每拍轮询
  # ——————————————————————————————————————————————————————————

  def poll(self):
    """
    排空 UART 缓冲区中的所有完整帧。
    返回最后一帧 DetectionFrame，无帧则返回 None。

    ★ 空帧 (num=0) 也会返回 DetectionFrame(0, [])，让调用方感知目标消失。
    """
    result = None
    while True:
      cmd, payload = recv_frame(self._uart)
      if cmd == 0x10:
        dets = parse_detections(payload) if payload else []
        self._detections = dets
        self._last_frame_ms = time.ticks_ms()
        self._lost_count = 0
        result = DetectionFrame(len(dets), dets)
      elif cmd is None:
        break  # 无更多完整帧
      # 忽略非 0x10 帧（如残留握手帧）

    if result is None:
      self._lost_count += 1

    return result

  # ——————————————————————————————————————————————————————————
  #                      握手
  # ——————————————————————————————————————————————————————————

  def handshake(self, retries=50, retry_ms=100):
    """
    同步握手。每拍调用一次直到返回 True 表示完成。
    改为非阻塞需 Robot 侧分拍调用；当前保留同步版供过渡期。
    """
    recv_timeout = retry_ms - 20
    if recv_timeout < 10:
      recv_timeout = 10

    self._ready = False
    self._failed = False
    self.flush()

    for _ in range(retries):
      send_frame(self._uart, 0x01)
      t0 = time.ticks_ms()
      while time.ticks_diff(time.ticks_ms(), t0) < recv_timeout:
        cmd, payload = recv_frame_handshake(self._uart, timeout_ms=recv_timeout)
        if cmd == 0x02:
          status = payload.decode('utf-8') if payload else ""
          if status == "200":
            send_frame(self._uart, 0x03)
            time.sleep_ms(10)
            self._ready = True
            self._last_frame_ms = time.ticks_ms()
            return True
          elif status == "400":
            self._failed = True
            return False
        elif cmd is not None:
          pass
        time.sleep_ms(5)

    return False

  # ——————————————————————————————————————————————————————————
  #                      状态查询
  # ——————————————————————————————————————————————————————————

  @property
  def detections(self):
    """最近一帧的检测列表（兼容旧 API）。"""
    return self._detections

  @property
  def has_target(self):
    """最近一帧是否有目标。"""
    return len(self._detections) > 0

  @property
  def timed_out(self):
    """距上次收帧是否超过超时阈值。"""
    if self._last_frame_ms == 0:
      return False  # 尚未收到任何帧
    return time.ticks_diff(time.ticks_ms(), self._last_frame_ms) > self._timeout_ms

  @property
  def is_ready(self):
    return self._ready

  @property
  def failed(self):
    return self._failed

  @property
  def lost_count(self):
    return self._lost_count

  def set_ready(self):
    self._ready = True
    self._last_frame_ms = time.ticks_ms()

  def flush(self):
    n = 0
    while self._uart.any() and n < 256:  # ★ 防止持续收帧时死循环
      self._uart.read(self._uart.any())
      n += 1
    self._detections = []
