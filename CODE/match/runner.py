"""
match/runner.py — 单车完赛编排（最终计划）

LEAVE → PICK → APPROACH → PRE_PUSH → PUSH → SCORE
  ↑── 转180° + 直行见目标 ── scored < N
                           scored ≥ N → HOME → DONE

运动学: 平移 = MotionControl.move；旋转 = [s,s,s]
"""

from time import ticks_ms, ticks_diff
from Motor import MotionControl
from config import CLS_LEFT, CLS_RIGHT
from app.fsm import IDLE, TRACK, COMPLETE, FAULT
from app.intent import START_TRACK, STOP, ABORT


def _wrap(a):
  while a > 180.0:
    a -= 360.0
  while a < -180.0:
    a += 360.0
  return a


class MatchRunner:
  """单车完赛编排器。"""

  OWNER = "MATCH"

  def __init__(self, robot, arbiter, tcs, cfg):
    self._robot = robot
    self._arb = arbiter
    self._tcs = tcs
    self._cfg = cfg

    self.phase = "IDLE"
    self.scored_count = 0
    self._sub = ""
    self._phase_ms = 0
    self._see_streak = 0
    self._remaining = []
    self._active_cls = None
    self._yaw_target = 0.0
    self._home_y2 = None  # leg2 yaw or None=skip
    self._home_deadline = 0

  # ————————————————————————————————————————————————————————
  #                      公有 API
  # ————————————————————————————————————————————————————————

  def start(self):
    """一键发车：ABORT 清场 → LEAVE。"""
    if self.phase not in ("IDLE", "DONE"):
      print("[MATCH] cannot start, phase=%s" % self.phase)
      return False
    self._robot.handle(ABORT)
    self._arb.force_brake()
    self.scored_count = 0
    c = self._cfg
    self._remaining = [int(x) for x in c.match_order]
    self._active_cls = None
    self._see_streak = 0
    self._sub = ""
    self._phase_ms = ticks_ms()
    # LEAVE 必须认所有类；避免上次 PICK 留下的 target_class 把目标滤掉
    c.tracking.target_class = 7
    c.match_allow = None
    self.phase = "LEAVE"
    self._arb.acquire(self.OWNER)
    print("[MATCH] START → LEAVE")
    return True

  def stop(self):
    """紧急停止。"""
    print("[MATCH] STOP")
    self.phase = "IDLE"
    self._sub = ""
    self._cfg.match_allow = None
    self._cfg.tracking.target_class = 7
    self._robot.handle(ABORT)
    self._arb.force_brake()

  # ————————————————————————————————————————————————————————
  #                      每拍 tick
  # ————————————————————————————————————————————————————————

  def tick(self, dt, sensors):
    if self.phase == "IDLE" or self.phase == "DONE":
      return
    if self.phase == "LEAVE":
      self._tick_leave(sensors)
    el    if self.phase == "PICK":
      self._tick_pick(sensors)
    elif self.phase == "APPROACH":
      self._tick_approach(sensors)
    elif self.phase == "PRE_PUSH":
      self._tick_pre_push()
    elif self.phase == "PUSH":
      self._tick_push(sensors)
    elif self.phase == "NEXT":
      self._tick_next(sensors)
    elif self.phase == "HOME":
      self._tick_home(sensors)

  # ————————————————————————————————————————————————————————
  #                      工具
  # ————————————————————————————————————————————————————————

  def _yaw(self):
    return self._robot._imu.get_yaw()

  def _yaw_err(self, target):
    return _wrap(target - self._yaw())

  def _write_move(self, speed, angle=0.0):
    self._arb.write(self.OWNER, MotionControl.move(speed, angle))

  def _write_spin(self, duty):
    """duty>0 与 HDG 同向（三轮同值=原地转）。"""
    d = float(duty)
    self._arb.write(self.OWNER, [d, d, d])

  def _brake(self):
    self._arb.force_brake()

  def _aligned(self, target):
    return abs(self._yaw_err(target)) <= float(self._cfg.align_tol_deg)

  def _spin_toward(self, target):
    err = self._yaw_err(target)
    if abs(err) <= float(self._cfg.align_tol_deg):
      self._arb.hold_brake(self.OWNER)
      return True
    s = float(self._cfg.drive_duty)
    if err < 0:
      s = -s
    self._write_spin(s)
    return False

  def _skip_or_home(self, why):
    """当前目标失败：剔除 cls 再 PICK；无剩余 → HOME。"""
    print("[MATCH] %s → skip cls=%s" % (why, self._active_cls))
    if self._active_cls is not None and self._active_cls in self._remaining:
      self._remaining.remove(self._active_cls)
    elif self._remaining:
      self._remaining.pop(0)
    if self._remaining:
      self._enter_pick()
    else:
      self._enter_home()

  def _seen_target(self, sensors, need=4):
    if sensors and sensors.get("has_target"):
      self._see_streak += 1
    else:
      self._see_streak = 0
    return self._see_streak >= need

  def _set_pick_class(self):
    """
    预赛 layout==0：认全部类 (7)。
    决赛：只认 _remaining（可多类），优先 match_order 头部。
    """
    c = self._cfg
    if int(c.start_layout) == 0 or not self._remaining:
      c.tracking.target_class = 7
      c.match_allow = None
      self._active_cls = None
    else:
      c.match_allow = list(self._remaining)
      c.tracking.target_class = int(self._remaining[0])
      self._active_cls = int(self._remaining[0])

  def _push_yaw(self):
    """决赛推向前目标航向；预赛 layout==0 返回 None（跳过 PRE_PUSH）。"""
    c = self._cfg
    if int(c.start_layout) == 0:
      return None
    cls = self._active_cls
    if cls is None:
      cls = c.tracking.target_class
    off = c.hdg_off_for(cls)
    # 三偏角全 0 → 预赛式直推
    if off == 0.0 and all(abs(float(x)) < 1e-6 for x in c.hdg_off):
      return None
    return _wrap(c.push_hdg_ref + off)

  def _home_plan(self):
    """
    返回 (leg1_yaw, leg2_yaw|None)。
    layout: 0/1 底边系 → 反场心一段；2 左下；3 右下；4 左边中跳过 leg2。
    """
    c = self._cfg
    href = float(c.push_hdg_ref)
    layout = int(c.start_layout)
    left = c.hdg_off_for(CLS_LEFT)
    right = c.hdg_off_for(CLS_RIGHT)
    if layout == 2:
      y1 = _wrap(href + left)
      y2 = _wrap(href + left + 45.0)
      return y1, y2
    if layout == 3:
      y1 = _wrap(href + right)
      y2 = _wrap(href + right - 45.0)
      return y1, y2
    if layout == 4:
      return _wrap(href + left), None
    return _wrap(href + 180.0), None

  def _enter_pick(self):
    self._see_streak = 0
    self._set_pick_class()
    # FAULT/COMPLETE 等需先回 IDLE，START_TRACK 才生效
    self._robot.handle(ABORT)
    self._arb.force_brake()
    self.phase = "PICK"
    self._phase_ms = ticks_ms()
    self._robot.handle(START_TRACK)
    print("[MATCH] → PICK cls=%s" % self._active_cls)

  def _enter_push(self):
    self._arb.acquire(self.OWNER)
    self._phase_ms = ticks_ms()
    self._tcs.reset_crossed()
    self.phase = "PUSH"
    self._sub = ""
    print("[MATCH] → PUSH")

  def _enter_home(self):
    y1, y2 = self._home_plan()
    self._yaw_target = y1
    self._home_y2 = y2
    self._home_deadline = ticks_ms() + int(self._cfg.home_timeout_ms)
    self._arb.acquire(self.OWNER)
    self._robot.handle(STOP)
    self.phase = "HOME"
    if self._tcs.on_line:
      self._sub = "LEAVE_LINE"
    else:
      self._sub = "LEG1_TURN"
    self._phase_ms = ticks_ms()
    print("[MATCH] → HOME sub=%s" % self._sub)

  # ————————————————————————————————————————————————————————
  #                      各 phase
  # ————————————————————————————————————————————————————————

  def _tick_leave(self, sensors):
    """直行出库直到稳定见目标 / 超时 → PICK。"""
    now = ticks_ms()
    if self._seen_target(sensors):
      print("[MATCH] LEAVE → see target")
      self._enter_pick()
      return
    if ticks_diff(now, self._phase_ms) > int(self._cfg.drive_timeout_ms):
      print("[MATCH] LEAVE timeout → PICK")
      self._enter_pick()
      return
    self._write_move(float(self._cfg.drive_duty), 0.0)

  def _tick_pick(self, sensors):
    st = self._robot.state
    if st == TRACK:
      t = sensors.get("target") if sensors else None
      if t is not None:
        self._active_cls = int(t[0])
      print("[MATCH] PICK → APPROACH cls=%s" % self._active_cls)
      self.phase = "APPROACH"
      self._phase_ms = ticks_ms()
    elif st in (IDLE, FAULT):
      # SEARCH/相机失败：跳过当前类，避免永久卡在 PICK
      self._skip_or_home("PICK state=%s" % st)

  def _tick_approach(self, sensors):
    st = self._robot.state
    if st == COMPLETE:
      t = sensors.get("target") if sensors else None
      if t is not None:
        self._active_cls = int(t[0])
      ty = self._push_yaw()
      if ty is None:
        self._enter_push()
      else:
        self._yaw_target = ty
        self._arb.acquire(self.OWNER)
        self._robot.handle(STOP)
        self.phase = "PRE_PUSH"
        self._phase_ms = ticks_ms()
        print("[MATCH] APPROACH → PRE_PUSH yaw=%.1f" % ty)
    elif st in (IDLE, FAULT):
      self._skip_or_home("APPROACH state=%s" % st)

  def _tick_pre_push(self):
    if self._spin_toward(self._yaw_target):
      self._enter_push()
      return
    # IMU 噪声 / 阈值过严时避免无限自旋
    if ticks_diff(ticks_ms(), self._phase_ms) > 3000:
      print("[MATCH] PRE_PUSH timeout → PUSH")
      self._enter_push()

  def _tick_push(self, sensors):
    now = ticks_ms()
    elapsed = ticks_diff(now, self._phase_ms)
    crossed = False
    if sensors:
      crossed = bool(sensors.get("tcs_crossed", False))
    timed_out = elapsed > int(self._cfg.push_timeout_ms)

    if crossed or timed_out:
      why = "yellow" if crossed else "timeout"
      print("[MATCH] PUSH → SCORE (%s %dms)" % (why, elapsed))
      self._brake()
      self._on_scored()
      return

    self._write_move(float(self._cfg.push_duty), 0.0)

  def _on_scored(self):
    self.scored_count += 1
    if self._active_cls is not None and self._active_cls in self._remaining:
      self._remaining.remove(self._active_cls)
    n = int(self._cfg.match_target_count)
    print("[MATCH] SCORE total=%d/%d rem=%s" % (
      self.scored_count, n, self._remaining))
    if self.scored_count >= n:
      self._enter_home()
      return
    # NEXT: 转 ~180° 再直行见目标
    self._yaw_target = _wrap(self._yaw() + 180.0)
    self._arb.acquire(self.OWNER)
    self._robot.handle(STOP)
    self.phase = "NEXT"
    self._sub = "SPIN"
    self._phase_ms = ticks_ms()
    self._see_streak = 0
    print("[MATCH] → NEXT SPIN")

  def _tick_next(self, sensors):
    now = ticks_ms()
    if self._sub == "SPIN":
      # 航向到位或开环超时兜底
      done = self._spin_toward(self._yaw_target)
      if done or ticks_diff(now, self._phase_ms) > int(self._cfg.next_spin_ms):
        self._sub = "DRIVE"
        self._phase_ms = now
        self._see_streak = 0
        print("[MATCH] NEXT → DRIVE")
      return
    # DRIVE
    if self._seen_target(sensors):
      print("[MATCH] NEXT see target → PICK")
      self._enter_pick()
      return
    if ticks_diff(now, self._phase_ms) > int(self._cfg.drive_timeout_ms):
      print("[MATCH] NEXT drive timeout → PICK")
      self._enter_pick()
      return
    self._write_move(float(self._cfg.drive_duty), 0.0)

  def _tick_home(self, sensors):
    now = ticks_ms()
    if ticks_diff(now, self._home_deadline) > 0:
      print("[MATCH] HOME timeout → DONE")
      self._finish()
      return

    crossed = bool(sensors.get("tcs_crossed", False)) if sensors else False
    on_line = self._tcs.on_line

    if self._sub == "LEAVE_LINE":
      if not on_line:
        self._sub = "LEG1_TURN"
        self._phase_ms = now
        print("[MATCH] HOME → LEG1_TURN")
        return
      self._write_move(-float(self._cfg.drive_duty), 0.0)
      return

    if self._sub == "LEG1_TURN":
      if self._spin_toward(self._yaw_target):
        self._sub = "LEG1_DRIVE"
        self._phase_ms = now
        self._tcs.reset_crossed()
        print("[MATCH] HOME → LEG1_DRIVE")
      return

    if self._sub == "LEG1_DRIVE":
      if crossed:
        if self._home_y2 is None:
          self._brake()
          self._finish()
          return
        # 两段 HOME：保留 MATCH owner，否则 BACKOFF/LEG2 写电机全丢
        self._arb.hold_brake(self.OWNER)
        self._sub = "BACKOFF"
        self._phase_ms = now
        print("[MATCH] HOME → BACKOFF")
        return
      self._write_move(float(self._cfg.drive_duty), 0.0)
      return

    if self._sub == "BACKOFF":
      if not on_line or ticks_diff(now, self._phase_ms) > 1500:
        self._yaw_target = self._home_y2
        self._sub = "LEG2_TURN"
        self._phase_ms = now
        print("[MATCH] HOME → LEG2_TURN")
        return
      self._write_move(-float(self._cfg.drive_duty), 0.0)
      return

    if self._sub == "LEG2_TURN":
      if self._spin_toward(self._yaw_target):
        self._sub = "LEG2_DRIVE"
        self._phase_ms = now
        self._tcs.reset_crossed()
        print("[MATCH] HOME → LEG2_DRIVE")
      return

    if self._sub == "LEG2_DRIVE":
      if crossed:
        self._finish()
        return
      self._write_move(float(self._cfg.drive_duty), 0.0)

  def _finish(self):
    self._brake()
    self._robot.handle(STOP)
    self.phase = "DONE"
    self._sub = ""
    print("[MATCH] DONE scored=%d" % self.scored_count)

  # ————————————————————————————————————————————————————————
  #                      状态查询
  # ————————————————————————————————————————————————————————

  @property
  def is_running(self):
    return self.phase not in ("IDLE", "DONE")

  @property
  def info(self):
    if self._sub:
      return "Match:%s/%s scored=%d" % (self.phase, self._sub, self.scored_count)
    return "Match:%s scored=%d" % (self.phase, self.scored_count)
