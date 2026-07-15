"""
match/runner.py — 单车完赛编排（最终计划）

LEAVE → PICK → APPROACH → PRE_PUSH → PUSH → SCORE
  ↑── 转180° + 直行见目标 ── scored < N
                           scored ≥ N → HOME → DONE

出库黄线忽略；进场离线后武装。
搜寻(PICK/NEXT直行)未推物时再黄线↑ = 出界 → RECOVER 掉头回场再搜。

运动学: 平移 = MotionControl.move；旋转 = [s,s,s]
"""

from time import ticks_ms, ticks_diff
from Motor import MotionControl
from ctrl import HeadingPID
from config import CLS_LEFT, CLS_RIGHT
from fsm import IDLE, TRACK, COMPLETE, FAULT
from fsm import START_TRACK, STOP, ABORT


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
    self._hold_yaw = 0.0
    self._home_y2 = None  # leg2 yaw or None=skip
    self._home_deadline = 0
    self._hdg_pid = HeadingPID(gains=cfg.heading)
    self._hdg_ms = ticks_ms()
    # 场界黄线：LEAVE 出库线忽略；离线后武装；搜寻中再↑则 RECOVER
    self._boundary_armed = False
    self._wait_off_line = False

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
    self._hold_yaw = self._yaw()  # 发车瞬间航向 = 出库直线目标
    self._hdg_pid.reset()
    self._hdg_ms = ticks_ms()
    self._boundary_armed = False
    self._wait_off_line = False
    self.phase = "LEAVE"
    self._arb.acquire(self.OWNER)
    print("[MATCH] START → LEAVE hold_yaw=%.1f" % self._hold_yaw)
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
    elif self.phase == "PICK":
      if self._check_search_boundary(sensors):
        return
      self._tick_pick(sensors)
    elif self.phase == "APPROACH":
      self._tick_approach(sensors)
    elif self.phase == "PRE_PUSH":
      self._tick_pre_push()
    elif self.phase == "PUSH":
      self._tick_push(sensors)
    elif self.phase == "NEXT":
      if self._sub == "DRIVE" and self._check_search_boundary(sensors):
        return
      self._tick_next(sensors)
    elif self.phase == "RECOVER":
      self._tick_recover(sensors)
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

  def _write_move_locked(self, speed, yaw_tgt):
    """平移 + 航向 P（同 HDG straight）：纠开环走歪。"""
    now = ticks_ms()
    dt = ticks_diff(now, self._hdg_ms) / 1000.0
    if dt <= 0.0 or dt > 0.5:
      dt = 0.02
    self._hdg_ms = now
    err = self._yaw_err(yaw_tgt)
    # err>0 表示需要沿正航向旋转；与 _spin_toward() 的符号约定一致。
    rot = self._hdg_pid.update(err, dt)
    fwd = MotionControl.move(float(speed), 0.0)
    duties = [
      self._clamp(fwd[0] + rot, -100.0, 100.0),
      self._clamp(fwd[1] + rot, -100.0, 100.0),
      self._clamp(fwd[2] + rot, -100.0, 100.0),
    ]
    self._arb.write(self.OWNER, duties)

  @staticmethod
  def _clamp(v, lo, hi):
    if v < lo:
      return lo
    if v > hi:
      return hi
    return v

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

  def _fault(self, why):
    """停止比赛但不伪装成 DONE，等待人工急停/重新发车。"""
    print("[MATCH] FAULT: %s" % why)
    self._robot.handle(ABORT)
    self._arb.force_brake()
    self.phase = "FAULT"
    self._sub = ""

  def _skip_or_home(self, why):
    """当前目标失败：尝试下一类；全部失败时进入 FAULT，禁止提前回库。"""
    print("[MATCH] %s → skip cls=%s" % (why, self._active_cls))
    if self._active_cls is not None and self._active_cls in self._remaining:
      self._remaining.remove(self._active_cls)
    elif self._remaining:
      self._remaining.pop(0)
    if self._remaining:
      self._enter_pick()
    else:
      self._fault("%s; no target class left (%d/%d scored)" % (
        why, self.scored_count, int(self._cfg.match_target_count)))

  def _seen_target(self, sensors, need=4):
    # 防抖必须按相机帧计数，不能把同一帧在 50Hz 主循环中重复累计。
    if sensors and sensors.get("new_frame"):
      if sensors.get("has_target"):
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
    # 出库/回场后可能仍压黄线：先等离线再武装场界
    self._arm_boundary_when_clear()
    self._robot.handle(START_TRACK)
    print("[MATCH] → PICK cls=%s armed=%s" % (
      self._active_cls, self._boundary_armed))

  def _arm_boundary_when_clear(self):
    """进场搜寻前：若还在黄线上则等离线后再认「出界」。"""
    self._boundary_armed = False
    if self._tcs.on_line:
      self._wait_off_line = True
      self._tcs.reset_crossed()
      print("[MATCH] boundary wait off-line")
    else:
      self._wait_off_line = False
      self._boundary_armed = True
      self._tcs.reset_crossed()
      print("[MATCH] boundary armed")

  def _check_search_boundary(self, sensors):
    """
    搜寻中（未推物）黄线 OFF→ON = 出场地 → RECOVER。
    返回 True 表示已切入 RECOVER，调用方应 return。
    """
    if self._wait_off_line:
      if not self._tcs.on_line:
        self._wait_off_line = False
        self._boundary_armed = True
        self._tcs.reset_crossed()
        print("[MATCH] boundary armed (cleared line)")
      return False
    if not self._boundary_armed:
      return False
    if sensors and sensors.get("tcs_crossed"):
      self._enter_recover()
      return True
    return False

  def _enter_recover(self):
    """出界：掉头回场，再搜物体。"""
    print("[MATCH] OUT OF BOUNDS (yellow) → RECOVER")
    self._robot.handle(ABORT)
    self._arb.force_brake()
    self._arb.acquire(self.OWNER)
    self._yaw_target = _wrap(self._yaw() + 180.0)
    self._boundary_armed = False
    self._wait_off_line = True
    self._tcs.reset_crossed()
    self.phase = "RECOVER"
    self._sub = "SPIN"
    self._phase_ms = ticks_ms()
    self._see_streak = 0
    self._hdg_pid.reset()

  def _enter_push(self):
    self._arb.acquire(self.OWNER)
    self._phase_ms = ticks_ms()
    self._hold_yaw = self._yaw()
    self._hdg_pid.reset()
    self._tcs.reset_crossed()
    self.phase = "PUSH"
    self._sub = "DRIVE"
    print("[MATCH] → PUSH")

  def _enter_home(self):
    y1, y2 = self._home_plan()
    self._yaw_target = y1
    self._home_y2 = y2
    self._home_deadline = ticks_ms() + int(self._cfg.home_timeout_ms)
    self._robot.handle(STOP)          # ★ handle 内部 force_brake → owner=None；先停再抢
    self._arb.acquire(self.OWNER)
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
    """锁发车航向直行，直到稳定见目标 / 超时 → PICK。"""
    now = ticks_ms()
    if self._seen_target(sensors):
      print("[MATCH] LEAVE → see target")
      self._enter_pick()
      return
    if ticks_diff(now, self._phase_ms) > int(self._cfg.drive_timeout_ms):
      print("[MATCH] LEAVE timeout → PICK")
      self._enter_pick()
      return
    self._write_move_locked(float(self._cfg.drive_duty), self._hold_yaw)

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
    elif ticks_diff(ticks_ms(), self._phase_ms) > 20000:
      # 20s 仍未找到目标：跳过当前类
      self._skip_or_home("PICK timeout")

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
        self._robot.handle(STOP)         # ★ handle 内部 force_brake → owner=None；先停再抢
        self._arb.acquire(self.OWNER)
        self.phase = "PRE_PUSH"
        self._phase_ms = ticks_ms()
        print("[MATCH] APPROACH → PRE_PUSH yaw=%.1f" % ty)
    elif st in (IDLE, FAULT):
      self._skip_or_home("APPROACH state=%s" % st)
    elif ticks_diff(ticks_ms(), self._phase_ms) > 15000:
      # 15s 仍未接近到位：跳过当前类
      self._skip_or_home("APPROACH timeout")

  def _tick_pre_push(self):
    if self._spin_toward(self._yaw_target):
      self._enter_push()
      return
    # IMU 噪声 / 阈值过严时避免无限自旋
    if ticks_diff(ticks_ms(), self._phase_ms) > 3000:
      self._fault("PRE_PUSH alignment timeout")

  def _tick_push(self, sensors):
    now = ticks_ms()
    elapsed = ticks_diff(now, self._phase_ms)
    if self._sub == "CLEAR":
      if elapsed >= int(self._cfg.push_clear_ms):
        print("[MATCH] PUSH → SCORE (yellow + clear %dms)" % elapsed)
        self._brake()
        self._on_scored()
        return
      self._write_move_locked(float(self._cfg.push_duty), self._hold_yaw)
      return

    crossed = False
    if sensors:
      crossed = bool(sensors.get("tcs_crossed", False))
    timed_out = elapsed > int(self._cfg.push_timeout_ms)

    if crossed:
      # 车底传感器上线后再前推一小段，确保推杆前方的物体完整离开黄线。
      self._sub = "CLEAR"
      self._phase_ms = now
      print("[MATCH] PUSH yellow → CLEAR")
      return

    if timed_out:
      print("[MATCH] PUSH timeout %dms — NOT scored" % elapsed)
      self._brake()
      self._skip_or_home("PUSH timeout")
      return

    self._write_move_locked(float(self._cfg.push_duty), self._hold_yaw)

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
    # match_order 是类别优先级，不是物品清单；同类可能有多个。
    if not self._remaining:
      self._remaining = [int(x) for x in self._cfg.match_order]
    # NEXT: 转 ~180° 再直行见目标（在场外，回场时黄线↑不算出界）
    self._yaw_target = _wrap(self._yaw() + 180.0)
    self._robot.handle(STOP)          # ★ handle 内部 force_brake → owner=None；先停再抢
    self._arb.acquire(self.OWNER)
    self._boundary_armed = False
    self._wait_off_line = True
    self._tcs.reset_crossed()
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
      timed_out = ticks_diff(now, self._phase_ms) > int(self._cfg.next_spin_ms)
      if done or timed_out:
        self._hold_yaw = self._yaw_target if done else self._yaw()
        self._hdg_pid.reset()
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
    self._write_move_locked(float(self._cfg.drive_duty), self._hold_yaw)

  def _tick_recover(self, sensors):
    """出界掉头：SPIN → DRIVE 见目标/超时 → PICK。"""
    now = ticks_ms()
    if self._sub == "SPIN":
      done = self._spin_toward(self._yaw_target)
      timed_out = ticks_diff(now, self._phase_ms) > int(self._cfg.next_spin_ms)
      if done or timed_out:
        self._hold_yaw = self._yaw_target if done else self._yaw()
        self._hdg_pid.reset()
        self._sub = "DRIVE"
        self._phase_ms = now
        self._see_streak = 0
        print("[MATCH] RECOVER → DRIVE")
      return
    if self._seen_target(sensors):
      print("[MATCH] RECOVER see target → PICK")
      self._enter_pick()
      return
    if ticks_diff(now, self._phase_ms) > int(self._cfg.drive_timeout_ms):
      print("[MATCH] RECOVER timeout → PICK")
      self._enter_pick()
      return
    # 回场途中可能压黄线：只等离线武装，不在此判出界
    if self._wait_off_line and not self._tcs.on_line:
      self._wait_off_line = False
      self._boundary_armed = True
      self._tcs.reset_crossed()
      print("[MATCH] boundary armed (recover)")
    self._write_move_locked(float(self._cfg.drive_duty), self._hold_yaw)

  def _tick_home(self, sensors):
    now = ticks_ms()
    if ticks_diff(now, self._home_deadline) > 0:
      self._fault("HOME timeout — gate not confirmed")
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
        self._hold_yaw = self._yaw_target
        self._hdg_pid.reset()
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
      self._write_move_locked(float(self._cfg.drive_duty), self._hold_yaw)
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
        self._hold_yaw = self._yaw_target
        self._hdg_pid.reset()
        self._sub = "LEG2_DRIVE"
        self._phase_ms = now
        self._tcs.reset_crossed()
        print("[MATCH] HOME → LEG2_DRIVE")
      return

    if self._sub == "LEG2_DRIVE":
      if crossed:
        self._finish()
        return
      self._write_move_locked(float(self._cfg.drive_duty), self._hold_yaw)

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
