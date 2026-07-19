"""
test_backoff.py — BACKOFF 后退+自旋 独立测试 (RETREAT / SPIN)

BACKOFF 逻辑:
  RETREAT: 后退至离线 (最少 N ms) → SPIN
  SPIN: 闭环 PD 自旋 180°(±3°×5帧) → HOME

用法:
  >>> import test_backoff
  >>> go()              # 启动 BACKOFF, 后退→PD自旋180°
  >>> tick() / run(20)  # 单步/连续
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
_OWNER = "BACKOFF_TEST"
_current_sub = "IDLE"; _exit_to = ""
_phase_ms = 0; _drive_duty = 50.0; _retreat_min_ms = 800
_spin_deg = 180.0; _yaw_target = 0.0; _spin_start_yaw = 0.0
_SPIN_YAW_TOL = 3.0; _spin_good = 0; _SPIN_GOOD_NEED = 5
_yaw_actuation_sign = -1.0; _hdg_pid = None
_bo_retreat = [0.0, 0.0, 0.0]
_ctrl_ms = 0; _rate_yaw = 0.0; _rate_ms = 0; _prev_yaw = 0.0

_tick_n = 0
def _tick_imu(_):
  global _tick_n
  try: _imu.update(); _tick_n += 1
  except Exception: pass

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

# ── 初始化 ──
def init():
  global _imu, _tkr, _motors, _arb, _hdg_pid, _bo_retreat
  print("[backoff] init IMU963...")
  _imu = ImuSensor(calibrate_samples=200, beta=0.05, model="963")
  _imu._gyro_scale = 1.0
  _tkr = ticker(1); _tkr.capture_list(_imu.raw); _tkr.callback(_tick_imu); _tkr.start(5)
  t0 = ticks_ms()
  while not _imu.is_calibrated:
    sleep_ms(10)
    if ticks_diff(ticks_ms(), t0) > 10000: break
  print("[backoff] IMU OK yaw=%.2f" % _yaw() if _imu.is_calibrated else "[backoff] 标定超时")
  _motors = MotionControl(); _arb = MotorArbiter(_motors)
  _hdg_pid = HeadingPID(kp=1.1, max_output=50.0, deadband=1.0, kd=0.08)
  d = -_drive_duty; mv = MotionControl.move_forward(d)
  _bo_retreat = [mv[0], mv[1], mv[2]]
  print("[backoff] 就绪. go() 后退+PD自旋180°")

# ── 入口 ──
def go(hz=20):
  global _current_sub, _exit_to, _phase_ms, _yaw_target, _spin_good
  _arb.acquire(_OWNER)
  _yaw_target = wrap_deg(_yaw() + 180.0)
  _current_sub = "RETREAT"; _exit_to = ""
  _phase_ms = ticks_ms()
  _spin_good = 0
  _hdg_pid.reset()
  print("[backoff] → RETREAT  target=%.1f°" % _yaw_target)
  _run_loop(hz)

def stop():
  _arb.force_brake(); global _current_sub; _current_sub = "IDLE"
  print("[backoff] 停止")

# ── 帧逻辑 ──
def tick():
  global _current_sub, _exit_to, _spin_start_yaw, _phase_ms, _spin_good
  if _current_sub == "IDLE": return
  now = ticks_ms(); elapsed = ticks_diff(now, _phase_ms)

  if _current_sub == "RETREAT":
    if elapsed >= _retreat_min_ms:
      err = _yaw_err(_yaw_target)
      dt = _control_dt(); rate = _yaw_rate()
      s = _yaw_actuation_sign * _hdg_pid.update(err, dt, rate)
      _spin_start_yaw = _yaw()
      _current_sub = "SPIN"
      print("[backoff] RETREAT → SPIN  err=%.1f°" % err)
      _arb.write(_OWNER, [s, s, s])
      return
    _arb.write(_OWNER, _bo_retreat)
    return

  if _current_sub == "SPIN":
    err = _yaw_err(_yaw_target)
    dt = _control_dt(); rate = _yaw_rate()
    s = _yaw_actuation_sign * _hdg_pid.update(err, dt, rate)
    _arb.write(_OWNER, [s, s, s])
    if abs(err) < _SPIN_YAW_TOL:
      _spin_good += 1
      if _spin_good >= _SPIN_GOOD_NEED:
        _arb.hold_brake(_OWNER)
        _exit_to = "HOME"; _current_sub = "IDLE"
        print("[backoff] SPIN done  err=%.1f° → %s" % (err, _exit_to))
        return
    else:
      _spin_good = 0
    if elapsed > 3000:  # 自旋超时 3s
      _exit_to = "HOME (timeout)"; _current_sub = "IDLE"; _arb.force_brake()
      print("[backoff] %s" % _exit_to)
      return

# ── 监控 ──
def mon():
  err = _yaw_err(_yaw_target) if _current_sub == "SPIN" else 0
  print("[backoff] sub=%s  yaw=%.1f  target=%.1f  err=%.1f°  good=%d/%d  exit=%s" % (
    _current_sub, _yaw(), _yaw_target, err, _spin_good, _SPIN_GOOD_NEED, _exit_to or "-"))

def _run_loop(hz=20):
  if _current_sub == "IDLE": print("[run] 请先 go()"); return
  dt = max(10, 1000 // hz); t_last = 0
  print("[run] %d Hz  sub=%s  Ctrl+C 停止" % (hz, _current_sub))
  try:
    while _current_sub != "IDLE":
      tick()
      now = ticks_ms()
      if ticks_diff(now, t_last) >= 300:
        t_last = now
        print("  sub=%s  yaw=%+.1f  dps=%.0f" % (_current_sub, _yaw(), _imu._gyro_dps))
      sleep_ms(dt)
  except KeyboardInterrupt: pass
  stop()

def run(hz=20): _run_loop(hz)

init()
