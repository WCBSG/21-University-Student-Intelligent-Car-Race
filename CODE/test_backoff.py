"""
backoff_test.py — BACKOFF 后退+转180° 状态独立测试 (RETREAT / SPIN)

用法:
  >>> import backoff_test
  >>> go()              # 启动 BACKOFF
  >>> tick() / run(20)  # 单步/连续
  >>> mon()
"""

from imu import ImuSensor
from motion import MotionControl, MotorArbiter, HeadingPID, wrap_deg
from time import ticks_ms, ticks_diff, sleep_ms
from smartcar import ticker

_imu = None; _tkr = None; _motors = None; _arb = None
_OWNER = "BACKOFF_TEST"
_current_sub = "IDLE"; _exit_to = ""
_phase_ms = 0
_drive_duty = 50.0; _retreat_min_ms = 800; _recover_timeout_ms = 9999
_spin_deg = 170.0; _yaw_target = 0.0
_yaw_actuation_sign = -1.0
_hdg_pid = None; _spin_start_yaw = 0.0
_bo_retreat = [0.0, 0.0, 0.0]
_rate_yaw = 0.0; _rate_ms = 0; _prev_yaw = 0.0

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

_ctrl_ms = 0

def init():
  global _imu, _tkr, _motors, _arb, _hdg_pid, _bo_retreat, _ctrl_ms
  print("[backoff] init IMU963...")
  _imu = ImuSensor(calibrate_samples=200, beta=0.05, model="963")
  _imu._gyro_scale = 1.0
  _tkr = ticker(1); _tkr.capture_list(_imu.raw); _tkr.callback(_tick); _tkr.start(5)
  t0 = ticks_ms()
  while not _imu.is_calibrated:
    sleep_ms(10)
    if ticks_diff(ticks_ms(), t0) > 10000: break
  print("[backoff] OK yaw=%.2f" % _yaw() if _imu.is_calibrated else "[backoff] 标定超时")
  _motors = MotionControl(); _arb = MotorArbiter(_motors)
  _hdg_pid = HeadingPID(kp=1.1, max_output=50.0, deadband=1.0, kd=0.08)
  d = -_drive_duty; mv = MotionControl.move_forward(d)
  _bo_retreat = [mv[0], mv[1], mv[2]]
  _ctrl_ms = ticks_ms()
  print("[backoff] 就绪. go() / run(20)")

def go():
  global _current_sub, _exit_to, _phase_ms, _yaw_target
  _arb.acquire(_OWNER)
  _yaw_target = wrap_deg(_yaw() + 180.0)
  _current_sub = "RETREAT"; _exit_to = ""
  _phase_ms = ticks_ms()
  _hdg_pid.reset()
  print("[backoff] → RETREAT  target=%.1f° (180° turn)" % _yaw_target)

def stop():
  _arb.force_brake(); global _current_sub; _current_sub = "IDLE"
  print("[backoff] 停止")

def tick():
  global _current_sub, _exit_to, _spin_start_yaw
  if _current_sub == "IDLE": return
  now = ticks_ms()
  elapsed = ticks_diff(now, _phase_ms)

  if _current_sub == "RETREAT":
    # 一直后退直到离线或超时
    if elapsed >= _retreat_min_ms:
      # 模拟离线 → 进 SPIN
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
    turned = abs(wrap_deg(_yaw() - _spin_start_yaw))
    if turned >= _spin_deg:
      _arb.hold_brake(_OWNER)
      _exit_to = "HOME"  # 或 FWD
      _current_sub = "IDLE"
      print("[backoff] SPIN done (%.1f°) → %s" % (turned, _exit_to))
      return
    if elapsed > _phase_ms + _recover_timeout_ms:
      _exit_to = "HOME (spin timeout)"
      _current_sub = "IDLE"; _arb.force_brake()
      print("[backoff] %s" % _exit_to)
      return
    err = _yaw_err(_yaw_target)
    dt = _control_dt(); rate = _yaw_rate()
    s = _yaw_actuation_sign * _hdg_pid.update(err, dt, rate)
    _arb.write(_OWNER, [s, s, s])

def mon():
  turned = abs(wrap_deg(_yaw() - _spin_start_yaw)) if _current_sub == "SPIN" else 0
  print("[backoff] sub=%s  yaw=%.1f  target=%.1f  turned=%.1f/%.0f  exit=%s" % (
    _current_sub, _yaw(), _yaw_target, turned, _spin_deg, _exit_to or "-"))

def run(hz=20):
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

init()
