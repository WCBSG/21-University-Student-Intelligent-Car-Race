"""
test_home.py — HOME 回库状态独立测试

子状态: LEAVE_LINE → LEG1_TURN → LEG1_DRIVE → BACKOFF → BACKOFF_TURN → LEG2_DRIVE → DONE

回库路径 (按 layout):
  layout=1(底边中): y1=180° → 单段直行到黄线
  layout=2(左下角): y1=90°  → 直行→压线→后退→转180°→直行→压线
  layout=3(右下角): y1=-90° → 直行→压线→后退→转180°→直行→压线

用法:
  >>> import test_home
  >>> on_line(True/False)  # 模拟黄线
  >>> go(layout=2)         # 启动 HOME
  >>> tick() / run(20)     # 单步/连续
  >>> mon()
"""

from imu import ImuSensor
from motion import MotionControl, MotorArbiter, HeadingPID, wrap_deg
from time import ticks_ms, ticks_diff, sleep_ms
from smartcar import ticker

# —— 热补丁 ——
import math as _math
def _patch_kinematics():
  MotionControl.move_forward = staticmethod(lambda speed: (
    lambda s = float(speed) * MotionControl._FWD_K: [s, -s, 0.0])())
  MotionControl.move_side = staticmethod(lambda speed: (
    lambda s = float(speed) * MotionControl._SIDE_K: [s, s, -2.0 * s])())
  def _move(speed, angle):
    r = _math.radians(-angle)
    c = _math.cos(r) / _math.sqrt(3)
    s = _math.sin(r) / 3
    return [speed*(s+c), speed*(s-c), speed*(-2*s)]
  MotionControl.move = staticmethod(_move)
_patch_kinematics()

# ── 全局 ──
_imu = None; _tkr = None; _motors = None; _arb = None
_OWNER = "HOME_TEST"
_current_sub = "IDLE"; _exit_to = ""
_phase_ms = 0; _timeout_ms = 20000

_yaw_target = 0.0; _home_y2 = None; _hold_yaw = 0.0
_drive_duty = 50.0; _retreat_min_ms = 800; _home_backoff_ms = 1500
_cross_ms = 1000  # 过线后再前进 1 秒防停在线上
_align_tol_deg = 10.0; _yaw_actuation_sign = -1.0
_hdg_pid = None; _hdg_pid_fwd = None; _turn_ok = 0
_FWD_LOCK_KP = 1.5  # 前向锁航向（MIN_DUTY 已关，不需太高）
_rate_yaw = 0.0; _rate_ms = 0; _prev_yaw = 0.0
_ctrl_ms = 0; _hdg_ms = 0

# 传感器
_tcs = None; _tcs_on_line = False; _tcs_ready = False; _tkr_tcs = None
_mock_on_line = False  # fallback: on_line(True/False) 模拟

_tick_n = 0
def _tick_imu(_):
  global _tick_n
  try: _imu.update(); _tick_n += 1
  except Exception: pass

def _tick_tcs(_):
  global _tcs_on_line
  if _tcs_ready:
    try:
      _tcs.crossed_yellow()
      _tcs_on_line = _tcs.on_line
    except Exception: pass

def _on_line():
  if _tcs_ready: return _tcs_on_line
  return _mock_on_line

def _yaw(): return _imu.get_yaw()
def _yaw_err(t): return wrap_deg(t - _yaw())
def _yaw_rate():
  global _rate_yaw, _rate_ms, _prev_yaw
  now = ticks_ms(); dt = ticks_diff(now, _rate_ms) / 1000.0; cur = _yaw()
  r = wrap_deg(cur - _prev_yaw) / dt if 0.001 < dt < 0.5 else 0.0
  _prev_yaw = cur; _rate_ms = now; _rate_yaw = r
  return r

def _control_dt():
  global _ctrl_ms
  now = ticks_ms(); dt = ticks_diff(now, _ctrl_ms) / 1000.0
  if dt <= 0.0 or dt > 0.5: dt = 0.05
  _ctrl_ms = now; return dt

def _clamp(v, lo, hi):
  if v < lo: return lo
  if v > hi: return hi
  return v

def on_line(v=True):
  """模拟黄线（仅 TCS 未初始化时生效）"""
  global _mock_on_line
  _mock_on_line = bool(v)

# ── 初始化 ──
def init(use_tcs=True):
  global _imu, _tkr, _motors, _arb, _hdg_pid, _hdg_pid_fwd, _ctrl_ms
  global _tcs, _tcs_ready, _tkr_tcs
  print("[home] init IMU963...")
  _imu = ImuSensor(calibrate_samples=200, beta=0.05, model="963")
  _imu._gyro_scale = 1.0
  _tkr = ticker(1); _tkr.capture_list(_imu.raw); _tkr.callback(_tick_imu); _tkr.start(5)
  t0 = ticks_ms()
  while not _imu.is_calibrated:
    sleep_ms(10)
    if ticks_diff(ticks_ms(), t0) > 10000: break
  print("[home] OK yaw=%.2f" % _yaw() if _imu.is_calibrated else "[home] 标定超时")
  # TCS (黄线)
  if use_tcs:
    try:
      from tcs3472 import TCS3472, make_i2c
      _tcs = TCS3472(make_i2c())
      _tcs.confirm_n = 2
      _tcs_ready = True
      _tkr_tcs = ticker(2); _tkr_tcs.callback(_tick_tcs); _tkr_tcs.start(20)
      print("[home] TCS OK")
    except Exception as e:
      print("[home] TCS 失败: %s" % e)
  _motors = MotionControl(); _arb = MotorArbiter(_motors)
  _hdg_pid = HeadingPID(kp=1.1, max_output=50.0, deadband=1.0, kd=0.08)
  _hdg_pid_fwd = HeadingPID(kp=_FWD_LOCK_KP, max_output=40.0, deadband=1.0, kd=0.10)
  _ctrl_ms = ticks_ms()
  print("[home] 就绪. go(layout=2) / on_line(bool) / run(20)")

# ── 路径规划 ──
def _plan(layout):
  """返回 (y1, y2): layout=1→(180,None) 2→(90,180) 3→(-90,180)"""
  if layout == 2: return 90.0, 180.0
  if layout == 3: return -90.0, 180.0
  return 180.0, None

# ── 自旋辅助 ──
def _spin_toward(target):
  global _turn_ok
  err = _yaw_err(target)
  if abs(err) <= _align_tol_deg and abs(_yaw_rate()) < 30.0:
    _turn_ok += 1
    if _turn_ok >= 8:
      _arb.hold_brake(_OWNER); _turn_ok = 0
      return True
    _arb.hold_brake(_OWNER)
    return False
  _turn_ok = 0
  dt = _control_dt(); rate = _yaw_rate()
  s = _yaw_actuation_sign * _hdg_pid.update(err, dt, rate)
  _arb.write(_OWNER, [s, s, s])
  return False

# ── 前向+航向锁 ──
def _write_fwd_locked(speed, hold_yaw):
  global _hdg_ms
  now = ticks_ms()
  dt = ticks_diff(now, _hdg_ms) / 1000.0
  if dt <= 0.0 or dt > 0.5: dt = 0.02
  _hdg_ms = now
  err = _yaw_err(hold_yaw); rate = _yaw_rate()
  rot = _yaw_actuation_sign * _hdg_pid_fwd.update(err, dt, rate)
  fwd = MotionControl.move_forward(float(speed))
  _arb.write(_OWNER, [_clamp(fwd[i]+rot, -100, 100) for i in range(3)], False)

# ── 入口 ──
def go(layout=2):
  global _yaw_target, _home_y2, _current_sub, _exit_to, _phase_ms, _turn_ok
  global _hold_yaw
  _arb.acquire(_OWNER)
  y1, y2 = _plan(layout)
  _yaw_target = float(y1); _home_y2 = float(y2) if y2 is not None else None
  _turn_ok = 0
  _hdg_pid.reset()
  if _on_line():
    _current_sub = "LEAVE_LINE"
    print("[home] → LEAVE_LINE (退离线)")
  else:
    _current_sub = "LEG1_TURN"
    print("[home] → LEG1_TURN  y1=%.1f  y2=%s" % (_yaw_target,
      ("%.1f" % _home_y2) if _home_y2 is not None else "-"))
  _exit_to = ""; _phase_ms = ticks_ms()
  _run_loop()

def stop():
  _arb.force_brake(); global _current_sub; _current_sub = "IDLE"
  print("[home] 停止")

# ── 帧逻辑 ──
def tick():
  global _current_sub, _exit_to, _phase_ms, _hold_yaw, _turn_ok, _yaw_target, _home_y2
  if _current_sub == "IDLE": return
  now = ticks_ms()
  elapsed = ticks_diff(now, _phase_ms)

  if elapsed > _timeout_ms and _current_sub not in ("LEG1_TURN", "BACKOFF_TURN"):
    _arb.force_brake()
    _exit_to = "FAULT (timeout)"; _current_sub = "IDLE"
    print("[home] %s" % _exit_to)
    return

  # ── LEAVE_LINE: 后退离黄线 ──
  if _current_sub == "LEAVE_LINE":
    if not _on_line():
      _current_sub = "LEG1_TURN"; _phase_ms = now
      print("[home] LEAVE_LINE → LEG1_TURN")
      return
    _arb.write(_OWNER, MotionControl.move_forward(-_drive_duty))
    return

  # ── LEG1_TURN: PD 转到 y1 ──
  if _current_sub == "LEG1_TURN":
    if _spin_toward(_yaw_target):
      _hold_yaw = _yaw_target; _hdg_pid.reset(); _hdg_pid_fwd.reset(); _turn_ok = 0
      _current_sub = "LEG1_DRIVE"; _phase_ms = now
      print("[home] LEG1_TURN → LEG1_DRIVE  hold=%.1f" % _hold_yaw)
    return

  # ── LEG1_DRIVE: 直行到压黄线 ──
  if _current_sub == "LEG1_DRIVE":
    if _on_line():
      if _home_y2 is None:
        _current_sub = "CROSS"; _phase_ms = now
        print("[home] LEG1_DRIVE → CROSS (crossing...)")
        return
      _arb.hold_brake(_OWNER)
      _current_sub = "BACKOFF"; _phase_ms = now
      print("[home] LEG1_DRIVE → BACKOFF (on yellow)")
      return
    _write_fwd_locked(_drive_duty, _hold_yaw)
    return

  # ── BACKOFF: 短退离线 ──
  if _current_sub == "BACKOFF":
    if elapsed < _retreat_min_ms:
      _arb.write(_OWNER, MotionControl.move_forward(-_drive_duty))
      return
    if not _on_line() or elapsed > _home_backoff_ms:
      _arb.hold_brake(_OWNER); _turn_ok = 0
      _current_sub = "BACKOFF_TURN"; _phase_ms = now
      print("[home] BACKOFF → BACKOFF_TURN  y2=%.1f" % _home_y2)
      return
    _arb.write(_OWNER, MotionControl.move_forward(-_drive_duty))
    return

  # ── BACKOFF_TURN: PD 转到 y2 ──
  if _current_sub == "BACKOFF_TURN":
    if _home_y2 is None:
      _exit_to = "DONE"; _current_sub = "IDLE"; _arb.force_brake()
      print("[home] %s" % _exit_to)
      return
    if _spin_toward(_home_y2):
      _hold_yaw = _home_y2; _hdg_pid.reset(); _hdg_pid_fwd.reset(); _turn_ok = 0
      _current_sub = "LEG2_DRIVE"; _phase_ms = now
      print("[home] BACKOFF_TURN → LEG2_DRIVE  hold=%.1f" % _hold_yaw)
    return

  # ── LEG2_DRIVE: 直行到压黄线 → CROSS ──
  if _current_sub == "LEG2_DRIVE":
    if _on_line():
      _current_sub = "CROSS"; _phase_ms = now
      print("[home] LEG2_DRIVE → CROSS (crossing...)")
      return
    _write_fwd_locked(_drive_duty, _hold_yaw)
    return

  # ── CROSS: 过线后继续前进 1 秒，防止停在线上 ──
  if _current_sub == "CROSS":
    if elapsed >= _cross_ms:
      _exit_to = "DONE"; _current_sub = "IDLE"; _arb.force_brake()
      print("[home] CROSS → %s" % _exit_to)
      return
    _write_fwd_locked(_drive_duty, _hold_yaw)
    return

# ── 监控 ──
def mon():
  print("[home] sub=%s  yaw=%.1f  target=%.1f  hold=%.1f  line=%s  y2=%s  exit=%s" % (
    _current_sub, _yaw(), _yaw_target, _hold_yaw, _on_line(),
    ("%.1f" % _home_y2) if _home_y2 is not None else "-",
    _exit_to or "-"))

def _run_loop(hz=20):
  if _current_sub == "IDLE": print("[run] 请先 go()"); return
  dt = max(10, 1000 // hz); t_last = 0
  print("[run] %d Hz  sub=%s  Ctrl+C 停止" % (hz, _current_sub))
  try:
    while _current_sub != "IDLE":
      tick()
      now = ticks_ms()
      if ticks_diff(now, t_last) >= 500:
        t_last = now
        print("  sub=%s  yaw=%+.1f  dps=%.0f  line=%s" % (
          _current_sub, _yaw(), _imu._gyro_dps, _on_line()))
      sleep_ms(dt)
  except KeyboardInterrupt: pass
  stop()

def run(hz=20): _run_loop(hz)

init()
