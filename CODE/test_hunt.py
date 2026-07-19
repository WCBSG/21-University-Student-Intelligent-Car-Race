"""
hunt_test.py — HUNT 搜索状态独立测试 (SPIN / FWD / TRACK)

用法:
  >>> import hunt_test
  >>> go()              # SPIN 模式开始
  >>> go_fwd()          # FWD 模式 (前进寻物)
  >>> see(cx, y2)       # 喂目标
  >>> hide()            # 目标丢失
  >>> tick()            # 单步
  >>> run(20)           # 20Hz 连续, Ctrl+C 停止
  >>> mon()             # 查看状态
"""

from imu import ImuSensor, wrap_deg as _wrap
from motion import MotionControl, MotorArbiter, HeadingPID
from time import ticks_ms, ticks_diff, ticks_add, sleep_ms
from smartcar import ticker

# ── 全局 ──
_imu = None; _tkr = None; _motors = None; _arb = None
_OWNER = "HUNT_TEST"
# 状态
_phase_ms = 0; _timeout_ms = 20000
_current_sub = "IDLE"  # SPIN | FWD | TRACK | DONE
_exit_to = ""
# 传感器
_mock_cx = 50.0; _mock_y2 = 0.0; _mock_has = False; _mock_new = False
# 参数
_search_speed = 15.0; _approach_speed = 55.0; _drive_duty = 50.0
_confirm_frames = 2; _lost_frames = 2
_stage_y2 = 75.0; _contact_y2 = 94.0
_yaw_actuation_sign = -1.0; _bearing_actuation_sign = 1.0
_match_mode = "final"; _center_fwd_ms = 4000
# PID
_bearing_pid = None; _track_ms = 0
# SPIN 内部
_search_dir = 1; _spin_acc = 0.0; _rev_start_yaw = 0.0
_SPIN_CIRCLE_DEG = 360.0
# TRACK 内部
_confirm_n = 0; _lost_n = 0; _see_streak = 0
# bearing rate
_prev_be = 0.0; _be_ms = 0
# yaw rate
_rate_yaw = 0.0; _rate_ms = 0; _prev_yaw = 0.0

_tick_n = 0
def _tick(_):
  global _tick_n
  try: _imu.update(); _tick_n += 1
  except Exception: pass

def _yaw(): return _imu.get_yaw()
def _yaw_err(t): return _wrap(t - _yaw())
def _yaw_rate():
  global _rate_yaw, _rate_ms, _prev_yaw
  now = ticks_ms(); dt = ticks_diff(now, _rate_ms) / 1000.0; cur = _yaw()
  r = _wrap(cur - _prev_yaw) / dt if 0.001 < dt < 0.5 else 0.0
  _prev_yaw = cur; _rate_ms = now; _rate_yaw = r
  return r

def see(cx=50.0, y2=60.0):
  global _mock_cx, _mock_y2, _mock_has, _mock_new
  _mock_cx = float(cx); _mock_y2 = float(y2)
  _mock_has = True; _mock_new = True

def hide():
  global _mock_has, _mock_new
  _mock_has = False; _mock_new = True

def _sensors():
  global _mock_new
  nf = _mock_new; _mock_new = False
  return {"new_frame": nf, "has_target": _mock_has,
          "target": [0,31,_mock_cx-10,_mock_y2-10,_mock_cx+10,_mock_y2,_mock_cx,0,0,_mock_y2] if _mock_has else None,
          "y2": _mock_y2}

def _clamp(v, lo, hi):
  if v < lo: return lo
  if v > hi: return hi
  return v

# ── 初始化 ──
def init():
  global _imu, _tkr, _motors, _arb, _bearing_pid
  print("[hunt] init IMU963...")
  _imu = ImuSensor(calibrate_samples=200, beta=0.05, model="963")
  _imu._gyro_scale = 1.0
  _tkr = ticker(1); _tkr.capture_list(_imu.raw); _tkr.callback(_tick); _tkr.start(5)
  t0 = ticks_ms()
  while not _imu.is_calibrated:
    sleep_ms(10)
    if ticks_diff(ticks_ms(), t0) > 10000: break
  print("[hunt] OK yaw=%.2f" % _yaw() if _imu.is_calibrated else "[hunt] 标定超时")
  _motors = MotionControl(); _arb = MotorArbiter(_motors)
  _bearing_pid = HeadingPID(kp=1.2, max_output=60.0, deadband=0.02, kd=0.05)
  print("[hunt] 就绪. go()=SPIN / go_fwd()=FWD / see(cx,y2) / run(20)")

# ── 入口 ──
def go(spin_dir=1):
  """SPIN 模式：原地旋转搜索"""
  global _current_sub, _phase_ms, _exit_to, _search_dir, _spin_acc, _rev_start_yaw
  global _confirm_n, _lost_n, _see_streak
  _arb.acquire(_OWNER)
  _current_sub = "SPIN"; _exit_to = ""
  _phase_ms = ticks_ms()
  _search_dir = 1 if spin_dir >= 0 else -1
  _spin_acc = 0.0; _rev_start_yaw = _yaw()
  _confirm_n = 0; _lost_n = 0; _see_streak = 0
  _bearing_pid.reset()
  print("[hunt] → SPIN  dir=%d  search_speed=%.0f" % (_search_dir, _search_speed))

def go_fwd():
  """FWD 模式：前进寻物"""
  global _current_sub, _phase_ms, _exit_to, _confirm_n, _see_streak
  _arb.acquire(_OWNER)
  _current_sub = "FWD"; _exit_to = ""
  _phase_ms = ticks_ms()
  _confirm_n = 0; _see_streak = 0
  _bearing_pid.reset()
  print("[hunt] → FWD  drive_duty=%.0f  timeout=%dms" % (_drive_duty, _center_fwd_ms))

def stop():
  _arb.force_brake(); global _current_sub; _current_sub = "IDLE"
  print("[hunt] 停止")

# ── 帧逻辑 ──
def tick():
  global _current_sub, _exit_to, _search_dir, _spin_acc, _rev_start_yaw
  global _confirm_n, _lost_n, _see_streak, _phase_ms
  global _prev_be, _be_ms

  if _current_sub == "IDLE": return
  sensors = _sensors()
  now = ticks_ms()
  has = bool(sensors["has_target"])
  t = sensors["target"]

  # 超时
  if ticks_diff(now, _phase_ms) > _timeout_ms:
    _arb.force_brake()
    _exit_to = "HUNT timeout → skip"
    _current_sub = "IDLE"
    print("[hunt] %s" % _exit_to)
    return

  # ── 确认/丢失计数 ──
  if sensors["new_frame"]:
    if has: _confirm_n += 1
    else: _confirm_n = 0

  # ── SPIN ──
  if _current_sub == "SPIN":
    if sensors["new_frame"] and _confirm_n >= _confirm_frames:
      _current_sub = "TRACK"; _lost_n = 0
      _bearing_pid.reset(); _track_ms = now
      print("[hunt] SPIN → TRACK (see target)")
      return
    if has:
      _arb.hold_brake(_OWNER)
      return
    y = _yaw(); d = _wrap(y - _rev_start_yaw)
    _spin_acc += abs(d); _rev_start_yaw = y
    if _spin_acc >= _SPIN_CIRCLE_DEG:
      _search_dir = -_search_dir
      if _search_dir == 0: _search_dir = 1
      _spin_acc = 0.0
      print("[hunt] SPIN flip → dir=%d" % _search_dir)
    s = _search_speed * _search_dir
    _arb.write(_OWNER, [s, s, s])
    return

  # ── FWD ──
  if _current_sub == "FWD":
    if ticks_diff(now, _phase_ms) > _center_fwd_ms:
      _current_sub = "SPIN"; _spin_acc = 0.0; _rev_start_yaw = _yaw()
      _confirm_n = 0; _phase_ms = now
      print("[hunt] FWD timeout → SPIN")
      return
    if sensors["new_frame"] and _confirm_n >= _confirm_frames:
      _current_sub = "TRACK"; _lost_n = 0
      _bearing_pid.reset(); _track_ms = now
      print("[hunt] FWD → TRACK (see target)")
      return
    fwd = MotionControl.move_forward(_drive_duty)
    _arb.write(_OWNER, [fwd[0], fwd[1], fwd[2]])
    return

  # ── TRACK ──
  if _current_sub == "TRACK":
    y2 = sensors["y2"]
    if sensors["new_frame"]:
      if has and y2 >= _stage_y2:
        _arb.hold_brake(_OWNER)
        if _match_mode != "pre":
          _exit_to = "ALIGN"
          print("[hunt] TRACK → %s (y2=%.1f >= stage=%.1f)" % (_exit_to, y2, _stage_y2))
        else:
          _exit_to = "PUSH"
          print("[hunt] TRACK → %s (contact)" % _exit_to)
        _current_sub = "DONE"
        return
      if has: _lost_n = 0
      else: _lost_n += 1
      if _lost_n >= _lost_frames:
        _search_dir = -_search_dir
        if _search_dir == 0: _search_dir = 1
        _current_sub = "SPIN"; _spin_acc = 0.0; _rev_start_yaw = _yaw()
        _confirm_n = 0; _lost_n = 0
        print("[hunt] TRACK lost → SPIN dir=%d" % _search_dir)
        return
    if t is None:
      _arb.hold_brake(_OWNER)
      return
    be = (float(t[6]) - 50.0) / 50.0
    dt = ticks_diff(now, _track_ms) / 1000.0
    if dt <= 0.0 or dt > 0.5: dt = 0.1
    _track_ms = now
    be_dt = ticks_diff(now, _be_ms) / 1000.0
    be_rate = (be - _prev_be) / be_dt if 0.001 < be_dt < 0.5 else 0.0
    _prev_be = be; _be_ms = now
    rot = _bearing_actuation_sign * _bearing_pid.update(be, dt, be_rate)
    fwd = MotionControl.move_forward(_approach_speed)
    _arb.write(_OWNER, [
      _clamp(fwd[0]+rot, -100, 100),
      _clamp(fwd[1]+rot, -100, 100),
      _clamp(fwd[2]+rot, -100, 100)])

# ── 监控 ──
def mon():
  print("[hunt] sub=%s  yaw=%.1f  has=%s  cx=%.1f  y2=%.1f  exit=%s" % (
    _current_sub, _yaw(), _mock_has, _mock_cx, _mock_y2, _exit_to or "-"))

def run(hz=20):
  if _current_sub == "IDLE": print("[run] 请先 go() 或 go_fwd()"); return
  dt = max(10, 1000 // hz); t_last = 0
  print("[run] %d Hz  sub=%s  Ctrl+C 停止" % (hz, _current_sub))
  try:
    while _current_sub not in ("IDLE", "DONE"):
      tick()
      now = ticks_ms()
      if ticks_diff(now, t_last) >= 500:
        t_last = now
        print("  sub=%s  yaw=%+.1f  cx=%.1f  y2=%.1f  has=%s  confirm=%d  lost=%d" % (
          _current_sub, _yaw(), _mock_cx, _mock_y2, _mock_has, _confirm_n, _lost_n))
      sleep_ms(dt)
  except KeyboardInterrupt: pass
  stop()

init()
