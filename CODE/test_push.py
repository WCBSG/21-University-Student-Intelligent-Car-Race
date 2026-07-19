"""
push_test.py — PUSH 推箱状态独立测试 (DRIVE / CORRECT)

用法:
  >>> import push_test
  >>> go(0, 66)        # 启动 PUSH: hold_yaw=0°, duty=66%
  >>> see(50, 90)      # 喂目标 cx=50% y2=90%
  >>> tick() / run(20) # 单步/连续
  >>> mon()            # 查看状态
"""

from imu import ImuSensor
from motion import MotionControl, MotorArbiter, HeadingPID, wrap_deg
from time import ticks_ms, ticks_diff, sleep_ms
from smartcar import ticker

_imu = None; _tkr = None; _motors = None; _arb = None
_OWNER = "PUSH_TEST"
_hold_yaw = 0.0; _push_duty = 66.0
_phase_ms = 0; _timeout_ms = 15000
_current_sub = "IDLE"; _exit_to = ""
_hdg_pid = None; _bearing_pid = None; _hdg_ms = 0; _ctrl_ms = 0
_mock_cx = 50.0; _mock_y2 = 0.0; _mock_has = False; _mock_new = False
_push_cx_left = 30.0; _push_cx_right = 78.0
_push_correct_duty = 30.0; _bearing_actuation_sign = 1.0; _yaw_actuation_sign = -1.0
_push_bad = 0; _push_bad_kind = ""; _push_seen = False; _push_last_cx = 50.0
_push_last_y2 = 0.0; _push_slipped = False; _lost_blind_ms = 600
_watch_frames = 2; _align_tol_deg = 10.0
_rate_yaw = 0.0; _rate_ms = 0; _prev_yaw = 0.0
_prev_be = 0.0; _be_ms = 0

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

def _clamp(v, lo, hi):
  if v < lo: return lo
  if v > hi: return hi
  return v

def see(cx=50.0, y2=90.0):
  global _mock_cx, _mock_y2, _mock_has, _mock_new
  _mock_cx = float(cx); _mock_y2 = float(y2)
  _mock_has = True; _mock_new = True

def hide():
  global _mock_has, _mock_new
  _mock_has = False; _mock_new = True

def _sensors():
  global _mock_new
  nf = _mock_new; _mock_new = False
  cx = _mock_cx; y2 = _mock_y2
  return {"new_frame": nf, "has_target": _mock_has,
          "target": [0,31,cx-10,y2-10,cx+10,y2,cx,0,0,y2] if _mock_has else None,
          "y2": y2}

# ── 初始化 ──
def init():
  global _imu, _tkr, _motors, _arb, _hdg_pid, _bearing_pid
  print("[push] init IMU963...")
  _imu = ImuSensor(calibrate_samples=200, beta=0.05, model="963")
  _imu._gyro_scale = 1.0
  _tkr = ticker(1); _tkr.capture_list(_imu.raw); _tkr.callback(_tick); _tkr.start(5)
  t0 = ticks_ms()
  while not _imu.is_calibrated:
    sleep_ms(10)
    if ticks_diff(ticks_ms(), t0) > 10000: break
  print("[push] OK yaw=%.2f" % _yaw() if _imu.is_calibrated else "[push] 标定超时")
  _motors = MotionControl(); _arb = MotorArbiter(_motors)
  _hdg_pid = HeadingPID(kp=1.1, max_output=50.0, deadband=1.0, kd=0.08)
  _bearing_pid = HeadingPID(kp=1.2, max_output=60.0, deadband=0.02, kd=0.05)
  print("[push] 就绪. go(hold_yaw, duty) / see(cx,y2) / run(20)")

# ── 控制 ──
def go(hold_yaw=0.0, duty=66):
  global _hold_yaw, _push_duty, _phase_ms, _current_sub, _exit_to
  global _push_bad, _push_bad_kind, _push_seen, _push_last_cx, _push_last_y2, _push_slipped
  _arb.acquire(_OWNER)
  _hold_yaw = float(hold_yaw); _push_duty = float(duty)
  _phase_ms = ticks_ms(); _current_sub = "DRIVE"; _exit_to = ""
  _push_bad = 0; _push_bad_kind = ""; _push_seen = False
  _push_last_cx = 50.0; _push_last_y2 = 0.0; _push_slipped = False
  _hdg_pid.reset(); _bearing_pid.reset()
  print("[push] → DRIVE  hold_yaw=%.1f  duty=%.0f" % (_hold_yaw, _push_duty))

def stop():
  _arb.force_brake(); global _current_sub; _current_sub = "IDLE"
  print("[push] 停止")

def tick():
  global _current_sub, _exit_to, _push_bad, _push_bad_kind, _push_seen
  global _push_last_cx, _push_last_y2, _push_slipped
  global _prev_be, _be_ms

  if _current_sub == "IDLE": return
  sensors = _sensors()
  now = ticks_ms()
  elapsed = ticks_diff(now, _phase_ms)

  if elapsed > _timeout_ms:
    _arb.force_brake()
    _exit_to = "HUNT (PUSH timeout)"
    _current_sub = "IDLE"
    print("[push] %s" % _exit_to)
    return

  # 帧监
  nf = sensors["new_frame"]
  t = sensors["target"]; has = bool(t) if t else False
  if nf:
    if t is None:
      if elapsed >= _lost_blind_ms and _push_seen and not _push_slipped:
        if _push_cx_left <= _push_last_cx <= _push_cx_right:
          if _push_last_y2 >= 75.0:  # occlusion OK
            _push_bad = 0; _push_bad_kind = ""
            # keep driving
          else:
            _push_bad_kind = "lost"; _push_bad += 1
        else:
          _push_bad_kind = "lost"; _push_bad += 1
      else:
        _push_bad_kind = "lost"; _push_bad += 1
    else:
      cx = float(t[6]); y2 = float(t[9])
      _push_seen = True; _push_last_cx = cx; _push_last_y2 = y2
      if _push_cx_left <= cx <= _push_cx_right:
        _push_bad = 0; _push_bad_kind = ""
      else:
        _push_slipped = True
        _push_bad_kind = "skew"; _push_bad += 1

  if _push_bad >= _watch_frames:
    if _push_bad_kind == "lost":
      _exit_to = "HUNT (reseek: lost)"
      _current_sub = "IDLE"; _arb.force_brake()
      print("[push] %s" % _exit_to)
      return
    elif _push_bad_kind == "skew":
      if _current_sub != "CORRECT":
        _current_sub = "CORRECT"; _bearing_pid.reset()
        print("[push] DRIVE → CORRECT (skew)")
  elif _current_sub == "CORRECT" and _push_bad_kind == "":
    _current_sub = "DRIVE"; _hdg_pid.reset()
    print("[push] CORRECT → DRIVE")

  # ── CORRECT ──
  if _current_sub == "CORRECT":
    if t is None: _arb.hold_brake(_OWNER); return
    bearing = (float(t[6]) - 50.0) / 50.0
    be_dt = ticks_diff(now, _be_ms) / 1000.0
    be_rate = (bearing - _prev_be) / be_dt if 0.001 < be_dt < 0.5 else 0.0
    _prev_be = bearing; _be_ms = now
    dt = ticks_diff(now, _ctrl_ms) / 1000.0
    if dt <= 0.0 or dt > 0.5: dt = 0.05
    _ctrl_ms = now
    rot = _bearing_actuation_sign * _bearing_pid.update(bearing, dt, be_rate)
    fwd = MotionControl.move_forward(_push_correct_duty)
    _arb.write(_OWNER, [_clamp(fwd[0]+rot, -100, 100),
                         _clamp(fwd[1]+rot, -100, 100),
                         _clamp(fwd[2]+rot, -100, 100)])
    return

  # ── DRIVE ──
  dt = ticks_diff(now, _hdg_ms) / 1000.0
  if dt <= 0.0 or dt > 0.5: dt = 0.02
  _hdg_ms = now
  err = _yaw_err(_hold_yaw); rate = _yaw_rate()
  rot = _yaw_actuation_sign * _hdg_pid.update(err, dt, rate)
  fwd = MotionControl.move_forward(_push_duty)
  _arb.write(_OWNER, [_clamp(fwd[0]+rot, -100, 100),
                       _clamp(fwd[1]+rot, -100, 100),
                       _clamp(fwd[2]+rot, -100, 100)])

def mon():
  print("[push] sub=%s  yaw=%.1f  hold=%.1f  cx=%.1f  y2=%.1f  bad=%d/%s  exit=%s" % (
    _current_sub, _yaw(), _hold_yaw, _mock_cx, _mock_y2, _push_bad, _push_bad_kind, _exit_to or "-"))

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
        print("  sub=%s  yaw=%+.1f  cx=%.1f  y2=%.1f  bad=%d/%s" % (
          _current_sub, _yaw(), _mock_cx, _mock_y2, _push_bad, _push_bad_kind))
      sleep_ms(dt)
  except KeyboardInterrupt: pass
  stop()

init()
