"""
match/runner.py — 单车完赛编排 (P1: 单件闭环)

MatchRunner 是比赛编排层，不修改 RobotFSM 状态表。
复用现有 SEARCH/TRACK Mode，PUSH 阶段直接接管 Arbiter。

用法 (REPL):
  from match.runner import MatchRunner
  match = MatchRunner(robot, arbiter, tcs, cfg)
  match.start()

state flow:
  IDLE → PICK → APPROACH → PUSH → SCORE → DONE
"""

from time import ticks_ms, ticks_diff
from app.fsm import IDLE, SEARCH, TRACK, COMPLETE
from app.intent import START_TRACK, STOP, ABORT


class MatchRunner:
  """单车完赛编排器。P1: 单件 PICK→APPROACH→PUSH→SCORE。"""

  OWNER = "MATCH"

  def __init__(self, robot, arbiter, tcs, cfg):
    self._robot = robot
    self._arb = arbiter
    self._tcs = tcs
    self._cfg = cfg

    self.phase = "IDLE"
    self.scored_count = 0
    self._push_start_ms = 0
    self._push_speed = 12.0  # PUSH 阶段低速(%)

  # ————————————————————————————————————————————————————————
  #                      公有 API
  # ————————————————————————————————————————————————————————

  def start(self):
    """一键发车：从 IDLE → PICK。"""
    if self.phase not in ("IDLE", "DONE"):
      print("[MATCH] cannot start, phase=%s" % self.phase)
      return False
    print("[MATCH] START → PICK")
    self.phase = "PICK"
    self._robot.handle(START_TRACK)  # 复用 SEARCH Mode
    return True

  def stop(self):
    """紧急停止。"""
    print("[MATCH] STOP")
    self.phase = "IDLE"
    self._robot.handle(ABORT)
    self._arb.force_brake()

  # ————————————————————————————————————————————————————————
  #                      每拍 tick
  # ————————————————————————————————————————————————————————

  def tick(self, dt, sensors):
    """主循环每拍调用。sensors 需包含 tcs_crossed。"""
    if self.phase == "IDLE" or self.phase == "DONE":
      return

    if self.phase == "PICK":
      self._tick_pick()
    elif self.phase == "APPROACH":
      self._tick_approach()
    elif self.phase == "PUSH":
      self._tick_push(sensors)

  # ————————————————————————————————————————————————————————
  #                      各 phase
  # ————————————————————————————————————————————————————————

  def _tick_pick(self):
    """等 FSM 从 SEARCH 进 TRACK。"""
    if self._robot.state == TRACK:
      print("[MATCH] PICK → APPROACH (target locked)")
      self.phase = "APPROACH"

  def _tick_approach(self):
    """
    等 FSM 从 TRACK 进 COMPLETE (bbox 触底 → 推杆接触)。
    RobotFSM.on_camera_frame 会在 y2 ≥ stop_bottom_pct 时自动转 COMPLETE。
    """
    if self._robot.state == COMPLETE:
      print("[MATCH] APPROACH → PUSH (contact)")
      self._arb.acquire(self.OWNER)
      self._push_start_ms = ticks_ms()
      self._tcs.reset_crossed()
      self.phase = "PUSH"

  def _tick_push(self, sensors):
    """
    低速直推。退出条件:
      - TCS 黄线上升沿 → SCORE (推出成功)
      - 超时 3 秒 → SCORE (兜底)
    """
    now = ticks_ms()
    elapsed = ticks_diff(now, self._push_start_ms)

    # 退出判据
    crossed = sensors.get("tcs_crossed", False) if sensors else False
    timed_out = elapsed > 3000

    if crossed:
      print("[MATCH] PUSH → SCORE (yellow crossed, %dms)" % elapsed)
      self._arb.force_brake()
      self.phase = "SCORE"
      self._on_scored()
      return

    if timed_out:
      print("[MATCH] PUSH → SCORE (timeout %dms)" % elapsed)
      self._arb.force_brake()
      self.phase = "SCORE"
      self._on_scored()
      return

    # 低速直推
    s = self._push_speed
    self._arb.write(self.OWNER, [s, s, s])

  def _on_scored(self):
    """SCORE: 记一件，停车。P1 不循环。"""
    self.scored_count += 1
    self._arb.force_brake()
    self._robot.handle(STOP)
    print("[MATCH] SCORED total=%d → DONE" % self.scored_count)
    self.phase = "DONE"

  # ————————————————————————————————————————————————————————
  #                      状态查询
  # ————————————————————————————————————————————————————————

  @property
  def is_running(self):
    return self.phase not in ("IDLE", "DONE")

  @property
  def info(self):
    return "Match:%s scored=%d" % (self.phase, self.scored_count)
