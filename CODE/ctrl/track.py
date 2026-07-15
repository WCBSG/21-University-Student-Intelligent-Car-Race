"""
ctrl/track.py — SEARCH / TRACK / COMPLETE Mode

- TrackSearchMode: spin | reverse；确认目标期间停转
- TrackApproachMode: bearing PI + 前进；到达由 FSM 判 y2
- CompleteMode: 停车等待 ABORT/STOP
Mode 不自行 transition。
"""

from time import ticks_ms, ticks_diff
from Motor import MotionControl
from HeadingController import HeadingPID
from app.mode import Mode, SEARCH, TRACK, COMPLETE, FAULT


def select_target(detections, cfg):
  """
  从检测列表选最佳目标。返回 detection tuple 或 None。

  过滤优先级:
    1. cfg.match_allow 为 list → 只接受这些 cls（决赛 remaining）
    2. 否则 target_class!=7 → 只接受该类
    3. target_class==7 且 match_allow is None → 全部类
  """
  if not detections:
    return None
  target_class = int(cfg.tracking.target_class)
  min_conf = cfg.tracking.min_confidence
  allow = getattr(cfg, "match_allow", None)
  candidates = []
  for d in detections:
    cls_id, score = d[0], d[1]
    if allow is not None:
      if cls_id not in allow:
        continue
    elif target_class != 7 and cls_id != target_class:
      continue
    if score < min_conf:
      continue
    if d[4] <= 0 or d[5] <= 0:
      continue
    candidates.append(d)
  if not candidates:
    return None
  candidates.sort(key=lambda x: (
    0 if x[0] == target_class or target_class == 7 else 1,
    -x[8]
  ))
  return candidates[0]


def fmt_target(target):
  if target is None:
    return "--"
  return "cls=%d area=%.0f%%" % (int(target[0]), target[8] / 100.0)


class TrackSearchMode(Mode):
  id = SEARCH

  def __init__(self, arbiter, imu, cfg, robot_ref):
    """robot_ref: RobotFSM，读 search_phase / 写 search_direction。"""
    self._arb = arbiter
    self._imu = imu
    self._cfg = cfg
    self._robot = robot_ref
    self._direction = 1
    self._rev_start_yaw = 0.0
    self._rev_acc = 0.0

  def enter(self):
    self._rev_acc = 0.0
    self._rev_start_yaw = self._imu.get_yaw()
    if self._robot.search_phase == "reverse":
      self._direction *= -1

  def exit(self):
    pass

  def begin_reverse(self):
    """FSM TRACK→SEARCH 丢失时、transition 之前调用。"""
    self._robot.search_phase = "reverse"
    # 方向翻转在 enter() 里做一次

  def update(self, dt, sensors):
    # 确认期间有目标 → 停转（去抖由 FSM 做）
    if sensors.get("has_target") and self._robot.search_phase != "reverse":
      self._arb.write(self.id, [0, 0, 0])
      return

    if self._robot.search_phase == "reverse":
      yaw = self._imu.get_yaw()
      delta = self._normalize(yaw - self._rev_start_yaw)
      self._rev_start_yaw = yaw
      self._rev_acc += abs(delta)
      rev_angle = self._cfg.tracking.reverse_angle
      if self._rev_acc >= rev_angle:
        self._robot.search_phase = "spin"
        self._rev_acc = 0.0

    speed = self._cfg.tracking.search_speed
    s = speed * self._direction
    self._arb.write(self.id, [s, s, s])

  @staticmethod
  def _normalize(angle):
    while angle > 180.0:
      angle -= 360.0
    while angle < -180.0:
      angle += 360.0
    return angle


class TrackApproachMode(Mode):
  id = TRACK

  def __init__(self, arbiter, imu, cfg):
    self._arb = arbiter
    self._imu = imu
    self._cfg = cfg
    self._pid = HeadingPID(gains=cfg.tracking_bearing)
    self._last_ms = ticks_ms()
    self.target_info = ""

  def enter(self):
    self._pid.reset()
    self._last_ms = ticks_ms()
    self.target_info = ""

  def exit(self):
    self._pid.reset()
    self.target_info = ""

  def update(self, dt, sensors):
    target = sensors.get("target")
    if target is None:
      # 短暂丢失：保持上一拍电机（不写）
      return

    self.target_info = fmt_target(target)
    cx, y2 = target[6], target[9]
    # 到达判定在 FSM.on_camera_frame；此处只控

    bearing_error = (cx - 50.0) / 50.0
    now = ticks_ms()
    real_dt = ticks_diff(now, self._last_ms) / 1000.0
    if real_dt <= 0.0 or real_dt > 0.5:
      real_dt = 0.1
    self._last_ms = now

    rotation = -self._pid.update(bearing_error, real_dt)
    forward_speed = self._cfg.tracking.approach_speed
    forward = MotionControl.move(forward_speed, 0.0)
    duties = [
      self._clamp(forward[0] + rotation, -100.0, 100.0),
      self._clamp(forward[1] + rotation, -100.0, 100.0),
      self._clamp(forward[2] + rotation, -100.0, 100.0),
    ]
    self._arb.write(self.id, duties)

  @staticmethod
  def _clamp(val, lo, hi):
    if val < lo: return lo
    if val > hi: return hi
    return val


class CompleteMode(Mode):
  id = COMPLETE

  def __init__(self, arbiter):
    self._arb = arbiter
    self.target_info = "COMPLETE"

  def enter(self):
    self._arb.write(self.id, [0, 0, 0])

  def update(self, dt, sensors):
    pass


class FaultMode(Mode):
  id = FAULT

  def __init__(self, arbiter):
    self._arb = arbiter

  def enter(self):
    self._arb.force_brake()

  def update(self, dt, sensors):
    pass
