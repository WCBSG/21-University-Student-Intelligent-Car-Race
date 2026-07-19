"""
test_push.py — PUSH 推箱状态独立测试 (DRIVE / CORRECT)

PUSH 逻辑:
  DRIVE:  直行前推 + 航向锁 + 视觉监护
  CORRECT: 目标偏出 cx 范围 → 慢速 bearing PID 纠偏
  退出: 黄线/超时/丢目标 → HUNT

用法:
  >>> import test_push
  >>> go(duty=66)       # 启动 PUSH
  >>> tick() / run(20)  # 单步/连续
  >>> mon()             # 查看状态
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
_OWNER = "PUSH_TEST"
_hold_yaw = 0.0; _push_duty = 66.0
_phase_ms = 0; _timeout_ms = 15000
_current_sub = "IDLE"; _exit_to = ""
_hdg_pid = None; _bearing_pid = None; _hdg_ms = 0; _ctrl_ms = 0

# 传感器
_mock_cx = 50.0; _mock_y2 = 0.0; _mock_has = False; _mock_new = False
_use_mock = True
_cam = None; _cam_new = False; _cam_last_ms = 0

# PUSH 参数 (同步 config.py)
_push_cx_left = 45.0; _push_cx_right = 55.0
_push_correct_duty = 30.0; _bearing_actuation_sign = 1.0; _yaw_actuation_sign = -1.0
_push_bad = 0; _push_bad_kind = ""; _push_seen = False; _push_last_cx = 50.0
_push_last_y2 = 0.0; _push_slipped = False; _lost_blind_ms = 600
_watch_frames = 2
_rate_yaw = 0.0; _rate_ms = 0; _prev_yaw = 0.0
_prev_be = 0.0; _be_ms = 0

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

def _clamp(v, lo, hi):
  if v < lo: return lo
  if v > hi: return hi
  return v

def see(cx=50.0, y2=90.0):
  global _mock_cx, _mock_y2, _mock_has, _mock_new, _use_mock
  _mock_cx = float(cx); _mock_y2 = float(y2)
  _mock_has = True; _mock_new = True; _use_mock = True

def hide():
  global _mock_has, _mock_new
  _mock_has = False; _mock_new = True

def _pick_best(detections):
  best = None; best_area = 0
  for d in detections:
    if int(d[1]) < 2: continue
    area = d[8] if len(d) > 8 else (d[3]*d[4])
    if area > best_area: best_area = area; best = d
  return best

def _poll_camera():
  global _cam_new, _cam_last_ms, _mock_has, _mock_new, _mock_cx, _mock_y2, _use_mock
  if _cam is None or not _cam.is_ready: return
  frame = _cam.poll()
  if frame is not None:
    _cam_last_ms = ticks_ms()
    if frame.has_target and frame.detections:
      t = _pick_best(frame.detections)
      if t is not None:
        _cam_new = True; _mock_has = True
        _mock_cx = float(t[6]); _mock_y2 = float(t[9])
        _mock_new = True; _use_mock = False
        return
    _cam_new = True; _mock_has = False; _mock_new = True
  elif _cam_last_ms and ticks_diff(ticks_ms(), _cam_last_ms) > 300:
    _mock_has = False; _cam_new = False

def _sensors():
  global _mock_new
  nf = _mock_new; _mock_new = False
  return {"new_frame": nf, "has_target": _mock_has,
          "target": [0,31,_mock_cx-10,_mock_y2-10,_mock_cx+10,_mock_y2,_mock_cx,0,0,_mock_y2] if _mock_has else None,
          "y2": _mock_y2}

# ── 初始化 ──
def init(use_cam=True):
  global _imu, _tkr, _motors, _arb, _hdg_pid, _bearing_pid, _cam
  print("[push] init IMU963...")
  _imu = ImuSensor(calibrate_samples=200, beta=0.05, model="963")
  _imu._gyro_scale = 1.0
  _tkr = ticker(1); _tkr.capture_list(_imu.raw); _tkr.callback(_tick_imu); _tkr.start(5)
  t0 = ticks_ms()
  while not _imu.is_calibrated:
    sleep_ms(10)
    if ticks_diff(ticks_ms(), t0) > 10000: break
  print("[push] IMU OK yaw=%.2f" % _yaw() if _imu.is_calibrated else "[push] 标定超时")
  _motors = MotionControl(); _arb = MotorArbiter(_motors)
  _hdg_pid = HeadingPID(kp=1.1, max_output=50.0, deadband=1.0, kd=0.08)
  _bearing_pid = HeadingPID(kp=1.2, max_output=60.0, deadband=0.02, kd=0.05)
  if use_cam:
    try:
      from machine import UART
      from camera import CameraRx
      _cam = CameraRx(UART(5, baudrate=460800), timeout_ms=5000)
      _cam.flush()
      for i in range(5):
        if _cam.handshake(retries=4, retry_ms=80):
          _cam.set_ready(); print("[push] CAM OK (try %d)" % (i+1)); break
        sleep_ms(100)
      if not _cam.is_ready: print("[push] CAM 握手失败 (用 mock)")
    except Exception as e: print("[push] CAM 失败: %s" % e)
  print("[push] 就绪. go(duty=66) / see(cx,y2) / run(20)")

# ── 入口 ──
def go(hold_yaw=None, duty=66, hz=20):
  global _hold_yaw, _push_duty, _phase_ms, _current_sub, _exit_to
  global _push_bad, _push_bad_kind, _push_seen, _push_last_cx, _push_last_y2, _push_slipped, _use_mock
  _arb.acquire(_OWNER)
  _hold_yaw = float(hold_yaw) if hold_yaw is not None else _yaw()
  _push_duty = float(duty); _phase_ms = ticks_ms()
  _current_sub = "DRIVE"; _exit_to = ""; _use_mock = True
  _push_bad = 0; _push_bad_kind = ""; _push_seen = False
  _push_last_cx = 50.0; _push_last_y2 = 0.0; _push_slipped = False
  _hdg_pid.reset(); _bearing_pid.reset()
  print("[push] → DRIVE  hold_yaw=%.1f  duty=%.0f" % (_hold_yaw, _push_duty))
  _run_loop(hz)

def stop():
  _arb.force_brake(); global _current_sub; _current_sub = "IDLE"
  print("[push] 停止")

# ── 帧逻辑 ──
def tick():
  global _current_sub, _exit_to, _push_bad, _push_bad_kind, _push_seen
  global _push_last_cx, _push_last_y2, _push_slipped
  global _prev_be, _be_ms, _hdg_ms, _ctrl_ms

  if _current_sub == "IDLE": return
  _poll_camera()
  sensors = _sensors()
  now = ticks_ms()
  elapsed = ticks_diff(now, _phase_ms)

  if elapsed > _timeout_ms:
    _arb.force_brake()
    _exit_to = "HUNT (timeout)"; _current_sub = "IDLE"
    print("[push] %s" % _exit_to)
    return

  nf = sensors["new_frame"]
  t = sensors["target"]; has = bool(t)
  if nf:
    if t is None:
      if elapsed >= _lost_blind_ms and _push_seen and not _push_slipped:
        if _push_cx_left <= _push_last_cx <= _push_cx_right:
          if _push_last_y2 >= 75.0:
            _push_bad = 0; _push_bad_kind = ""
          else: _push_bad_kind = "lost"; _push_bad += 1
        else: _push_bad_kind = "lost"; _push_bad += 1
      else: _push_bad_kind = "lost"; _push_bad += 1
    else:
      cx = float(t[6]); y2 = float(t[9])
      _push_seen = True; _push_last_cx = cx; _push_last_y2 = y2
      if _push_cx_left <= cx <= _push_cx_right:
        _push_bad = 0; _push_bad_kind = ""
      else:
        _push_slipped = True; _push_bad_kind = "skew"; _push_bad += 1

  if _push_bad >= _watch_frames:
    if _push_bad_kind == "lost":
      _exit_to = "HUNT (reseek)"; _current_sub = "IDLE"; _arb.force_brake()
      print("[push] %s" % _exit_to); return
    elif _push_bad_kind == "skew":
      if _current_sub != "CORRECT":
        _current_sub = "CORRECT"; _bearing_pid.reset()
        print("[push] DRIVE → CORRECT")
  elif _current_sub == "CORRECT" and _push_bad_kind == "":
    _current_sub = "DRIVE"; _hdg_pid.reset()
    print("[push] CORRECT → DRIVE")

  # ── CORRECT: PD侧移闭环, 减速前推 ──
  if _current_sub == "CORRECT":
    if t is None: _arb.hold_brake(_OWNER); return
    cx = float(t[6])
    err_cx = cx - 50.0  # cx>50→右偏→右移
    d_cx = cx - _push_last_cx
    lateral = _clamp(err_cx * 0.6 - d_cx * 0.5, -60.0, 60.0)
    fwd = MotionControl.move_forward(12.0)  # 减速前推
    side = MotionControl.move_side(lateral) if abs(lateral) > 1e-6 else (0.0, 0.0, 0.0)
    _arb.write(_OWNER, [_clamp(fwd[i]+side[i], -100, 100) for i in range(3)], False)
    return

  # ── DRIVE: 全速前推 + 航向锁 ──
  dt = ticks_diff(now, _hdg_ms) / 1000.0
  if dt <= 0.0 or dt > 0.5: dt = 0.02
  _hdg_ms = now
  err = _yaw_err(_hold_yaw); rate = _yaw_rate()
  rot = _yaw_actuation_sign * _hdg_pid.update(err, dt, rate)
  fwd = MotionControl.move_forward(_push_duty)
  _arb.write(_OWNER, [_clamp(fwd[i]+rot, -100, 100) for i in range(3)], False)

# ── 监控 ──
def mon():
  print("[push] sub=%s  yaw=%.1f  hold=%.1f  cx=%.1f  y2=%.1f  bad=%d/%s  exit=%s" % (
    _current_sub, _yaw(), _hold_yaw, _mock_cx, _mock_y2, _push_bad, _push_bad_kind, _exit_to or "-"))

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
        print("  sub=%s  yaw=%+.1f  cx=%.1f  y2=%.1f  bad=%d/%s" % (
          _current_sub, _yaw(), _mock_cx, _mock_y2, _push_bad, _push_bad_kind))
      sleep_ms(dt)
  except KeyboardInterrupt: pass
  stop()

def run(hz=20): _run_loop(hz)

init()
