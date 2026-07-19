"""
home_test.py — HOME 回库状态独立测试

子状态: LEAVE_LINE→LEG1_TURN→LEG1_DRIVE→BACKOFF→BACKOFF_TURN→LEG2_DRIVE→DONE

用法:
  >>> import home_test
  >>> go(y1=90, y2=180)  # y1=第一段目标航向, y2=第二段(None=单段回库)
  >>> tick() / run(20)    # 单步/连续
  >>> on_line(True)       # 模拟压黄线
  >>> mon()
"""

from imu import ImuSensor
from motion import MotionControl, MotorArbiter, HeadingPID, wrap_deg
from time import ticks_ms, ticks_diff, sleep_ms
from smartcar import ticker

_imu = None; _tkr = None; _motors = None; _arb = None
_OWNER = "HOME_TEST"
_current_sub = "IDLE"; _exit_to = ""
_phase_ms = 0; _timeout_ms = 20000
_yaw_target = 0.0; _home_y2 = None; _hold_yaw = 0.0
_drive_duty = 50.0; _retreat_min_ms = 800; _home_backoff_ms = 1500
_align_tol_deg = 10.0; _yaw_actuation_sign = -1.0
_hdg_pid = None; _home_turn_ok = 0
_rate_yaw = 0.0; _rate_ms = 0; _prev_yaw = 0.0
_ctrl_ms = 0; _hdg_ms = 0
_mock_on_line = False  # 模拟黄线

_tick_n = 0
def _tick(_):
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

def on_line(v=True):
  global _mock_on_line
  _mock_on_line = bool(v)

# ── 初始化 ──
def init():
  global _imu, _tkr, _motors, _arb, _hdg_pid, _ctrl_ms
  print("[home] init IMU963...")
  _imu = ImuSensor(calibrate_samples=200, beta=0.05, model="963")
  _imu._gyro_scale = 1.0
  _tkr = ticker(1); _tkr.capture_list(_imu.raw); _tkr.callback(_tick); _tkr.start(5)
  t0 = ticks_ms()
  while not _imu.is_calibrated:
    sleep_ms(10)
    if ticks_diff(ticks_ms(), t0) > 10000: break
  print("[home] OK yaw=%.2f" % _yaw() if _imu.is_calibrated else "[home] 标定超时")
  _motors = MotionControl(); _arb = MotorArbiter(_motors)
  _hdg_pid = HeadingPID(kp=1.1, max_output=50.0, deadband=1.0, kd=0.08)
  _ctrl_ms = ticks_ms()
  print("[home] 就绪. go(y1, y2) / on_line(True|False) / run(20)")

# ── 控制 ──
def go(y1=90.0, y2=None):
  global _yaw_target, _home_y2, _current_sub, _exit_to, _phase_ms, _home_turn_ok
  global _hold_yaw
  _arb.acquire(_OWNER)
  _yaw_target = float(y1); _home_y2 = float(y2) if y2 is not None else None
  _home_turn_ok = 0
  if _mock_on_line:
    _current_sub = "LEAVE_LINE"; print("[home] → LEAVE_LINE (back off line)")
  else:
    _current_sub = "LEG1_TURN"; print("[home] → LEG1_TURN  y1=%.1f" % _yaw_target)
  _exit_to = ""; _phase_ms = ticks_ms()
  _hdg_pid.reset()

def stop():
  _arb.force_brake(); global _current_sub; _current_sub = "IDLE"
  print("[home] 停止")

def _spin_toward(target):
  global _home_turn_ok
  err = _yaw_err(target)
  if abs(err) <= _align_tol_deg:
    _home_turn_ok += 1
    if _home_turn_ok >= 3:
      _arb.hold_brake(_OWNER); _home_turn_ok = 0
      return True
    _arb.hold_brake(_OWNER)
    return False
  _home_turn_ok = 0
  dt = _control_dt(); rate = _yaw_rate()
  s = _yaw_actuation_sign * _hdg_pid.update(err, dt, rate)
  _arb.write(_OWNER, [s, s, s])
  return False

def tick():
  global _current_sub, _exit_to, _phase_ms, _hold_yaw, _home_turn_ok, _yaw_target
  if _current_sub == "IDLE": return
  now = ticks_ms()

  if ticks_diff(now, _phase_ms) > _timeout_ms:
    if _current_sub != "LEG1_TURN" and _current_sub != "BACKOFF_TURN":
      _arb.force_brake()
      _exit_to = "FAULT (HOME timeout)"
      _current_sub = "IDLE"
      print("[home] %s" % _exit_to)
      return

  # ── LEAVE_LINE: 后退离黄线 ──
  if _current_sub == "LEAVE_LINE":
    if not _mock_on_line:
      _current_sub = "LEG1_TURN"; _phase_ms = now
      print("[home] LEAVE_LINE → LEG1_TURN")
      return
    d = -_drive_duty
    mv = MotionControl.move_forward(d)
    _arb.write(_OWNER, mv)
    return

  # ── LEG1_TURN: 转到 y1 ──
  if _current_sub == "LEG1_TURN":
    if _spin_toward(_yaw_target):
      _hold_yaw = _yaw_target; _hdg_pid.reset(); _home_turn_ok = 0
      _current_sub = "LEG1_DRIVE"; _phase_ms = now
      print("[home] LEG1_TURN → LEG1_DRIVE  hold=%.1f" % _hold_yaw)
    return

  # ── LEG1_DRIVE: 直行到压黄线 ──
  if _current_sub == "LEG1_DRIVE":
    if _mock_on_line:
      if _home_y2 is None:
        _arb.force_brake()
        _exit_to = "DONE (single leg)"
        _current_sub = "IDLE"
        print("[home] %s" % _exit_to)
        return
      _arb.hold_brake(_OWNER)
      _current_sub = "BACKOFF"; _phase_ms = now
      print("[home] LEG1_DRIVE → BACKOFF (on yellow)")
      return
    if abs(_yaw_err(_hold_yaw)) > 12.0:
      _spin_toward(_hold_yaw)
      return
    dt = ticks_diff(now, _hdg_ms) / 1000.0
    if dt <= 0.0 or dt > 0.5: dt = 0.02
    _hdg_ms = now
    err = _yaw_err(_hold_yaw); rate = _yaw_rate()
    rot = _yaw_actuation_sign * _hdg_pid.update(err, dt, rate)
    fwd = MotionControl.move_forward(_drive_duty)
    _arb.write(_OWNER, [max(-100,min(100,fwd[0]+rot)),
                         max(-100,min(100,fwd[1]+rot)),
                         max(-100,min(100,fwd[2]+rot))])
    return

  # ── BACKOFF: 短退离黄线 ──
  if _current_sub == "BACKOFF":
    elapsed = ticks_diff(now, _phase_ms)
    if elapsed < _retreat_min_ms:
      d = -_drive_duty
      mv = MotionControl.move_forward(d)
      _arb.write(_OWNER, mv)
      return
    if not _mock_on_line or elapsed > _home_backoff_ms:
      _arb.hold_brake(_OWNER); _home_turn_ok = 0
      _current_sub = "BACKOFF_TURN"; _phase_ms = now
      print("[home] BACKOFF → BACKOFF_TURN  y2=%.1f" % _home_y2)
      return
    d = -_drive_duty
    mv = MotionControl.move_forward(d)
    _arb.write(_OWNER, mv)
    return

  # ── BACKOFF_TURN: 转到 y2 ──
  if _current_sub == "BACKOFF_TURN":
    if _home_y2 is None:
      _exit_to = "DONE"; _current_sub = "IDLE"; _arb.force_brake()
      print("[home] %s" % _exit_to)
      return
    if _spin_toward(_home_y2):
      _hold_yaw = _home_y2; _yaw_target = _home_y2
      _hdg_pid.reset(); _home_turn_ok = 0
      _current_sub = "LEG2_DRIVE"; _phase_ms = now
      print("[home] BACKOFF_TURN → LEG2_DRIVE  hold=%.1f" % _hold_yaw)
    return

  # ── LEG2_DRIVE: 直行到压黄线 → DONE ──
  if _current_sub == "LEG2_DRIVE":
    if _mock_on_line:
      _exit_to = "DONE"; _current_sub = "IDLE"; _arb.force_brake()
      print("[home] LEG2_DRIVE → %s (on yellow)" % _exit_to)
      return
    if abs(_yaw_err(_hold_yaw)) > 12.0:
      _spin_toward(_hold_yaw)
      return
    dt = ticks_diff(now, _hdg_ms) / 1000.0
    if dt <= 0.0 or dt > 0.5: dt = 0.02
    _hdg_ms = now
    err = _yaw_err(_hold_yaw); rate = _yaw_rate()
    rot = _yaw_actuation_sign * _hdg_pid.update(err, dt, rate)
    fwd = MotionControl.move_forward(_drive_duty)
    _arb.write(_OWNER, [max(-100,min(100,fwd[0]+rot)),
                         max(-100,min(100,fwd[1]+rot)),
                         max(-100,min(100,fwd[2]+rot))])
    return

def mon():
  print("[home] sub=%s  yaw=%.1f  target=%.1f  hold=%.1f  line=%s  y2=%s  exit=%s" % (
    _current_sub, _yaw(), _yaw_target, _hold_yaw, _mock_on_line,
    ("%.1f" % _home_y2) if _home_y2 is not None else "-",
    _exit_to or "-"))

def run(hz=20):
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
          _current_sub, _yaw(), _imu._gyro_dps, _mock_on_line))
      sleep_ms(dt)
  except KeyboardInterrupt: pass
  stop()

init()
