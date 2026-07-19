"""
test_hunt.py — HUNT 搜索状态独立测试 (SPIN / FWD / TRACK)

两阶段搜索:
  SPIN:  原地自旋搜索目标（每360°反转方向）
  TRACK: 看到目标→视觉 bearing PID 追踪接近，y2≥stage→ALIGN/PUSH

用法:
  >>> import test_hunt
  >>> go()              # SPIN 模式 + 自动 run(20)
  >>> see(cx, y2)       # 手动喂目标（测试用）
  >>> hide()            # 目标丢失
  >>> tick()            # 单步
  >>> mon()             # 查看状态
  >>> cam_test(10)      # 摄像头检测 10s
"""

from imu import ImuSensor
from motion import MotionControl, MotorArbiter, HeadingPID, wrap_deg as _wrap
from time import ticks_ms, ticks_diff, sleep_ms
from smartcar import ticker

# —— 热补丁: 运动学, 等 motion.py 重烧后可删 ——
def _patch_kinematics():
  MotionControl.move_forward = staticmethod(lambda speed: (
    lambda s = float(speed) * MotionControl._FWD_K: [s, -s, 0.0])())
  MotionControl.move_side = staticmethod(lambda speed: (
    lambda s = float(speed) * MotionControl._SIDE_K: [s, s, -2.0 * s])())

_patch_kinematics()

# ── 全局 ──
_imu = None; _tkr = None; _motors = None; _arb = None
_OWNER = "HUNT_TEST"
# 状态
_phase_ms = 0; _timeout_ms = 20000
_current_sub = "IDLE"  # SPIN | TRACK | DONE
_exit_to = ""
# 传感器 (mock + real)
_mock_cx = 50.0; _mock_y2 = 0.0; _mock_has = False; _mock_new = False
_use_mock = True  # True=用 mock see()/hide(), False=用真实摄像头
_cam = None; _cam_new = False; _cam_last_ms = 0
# 参数
_search_speed = 15.0; _approach_speed = 55.0
_confirm_frames = 2; _lost_frames = 2
_stage_y2 = 75.0; _contact_y2 = 94.0
_bearing_actuation_sign = 1.0
_match_mode = "final"
_PUSH_HDG = {0: 90.0, 1: 0.0, 2: -90.0}  # 推箱方向, 接近方向=+180°
_ALIGN_YAW_TOL = 25.0  # yaw 误差在此范围内可跳过 ALIGN 直进 PUSH
# PID
_bearing_pid = None; _track_ms = 0
# SPIN 内部
_search_dir = 1; _spin_acc = 0.0; _rev_start_yaw = 0.0
_SPIN_CIRCLE_DEG = 360.0
# TRACK 内部
_confirm_n = 0; _lost_n = 0
# bearing rate
_prev_be = 0.0; _be_ms = 0
# yaw rate
_rate_yaw = 0.0; _rate_ms = 0; _prev_yaw = 0.0

_tick_n = 0
def _tick_imu(_):
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
  """手动喂目标"""
  global _mock_cx, _mock_y2, _mock_has, _mock_new, _use_mock
  _mock_cx = float(cx); _mock_y2 = float(y2)
  _mock_has = True; _mock_new = True; _use_mock = True

def hide():
  global _mock_has, _mock_new
  _mock_has = False; _mock_new = True

def _pick_best(detections):
  best = None; best_area = 0
  for d in detections:
    conf = int(d[1])
    if conf < 2: continue
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

def _clamp(v, lo, hi):
  if v < lo: return lo
  if v > hi: return hi
  return v

# ── 初始化 ──
def init(use_cam=True):
  global _imu, _tkr, _motors, _arb, _bearing_pid, _cam
  print("[hunt] init IMU963...")
  _imu = ImuSensor(calibrate_samples=200, beta=0.05, model="963")
  _imu._gyro_scale = 1.0
  _tkr = ticker(1); _tkr.capture_list(_imu.raw); _tkr.callback(_tick_imu); _tkr.start(5)
  t0 = ticks_ms()
  while not _imu.is_calibrated:
    sleep_ms(10)
    if ticks_diff(ticks_ms(), t0) > 10000: break
  print("[hunt] IMU OK yaw=%.2f" % _yaw() if _imu.is_calibrated else "[hunt] 标定超时")
  _motors = MotionControl(); _arb = MotorArbiter(_motors)
  _bearing_pid = HeadingPID(kp=1.2, max_output=60.0, deadband=0.02, kd=0.05)
  # Camera
  if use_cam:
    try:
      from machine import UART
      from camera import CameraRx
      _cam = CameraRx(UART(5, baudrate=460800), timeout_ms=5000)
      _cam.flush()
      for i in range(5):
        if _cam.handshake(retries=4, retry_ms=80):
          _cam.set_ready(); print("[hunt] CAM OK (try %d)" % (i+1)); break
        sleep_ms(100)
      if not _cam.is_ready: print("[hunt] CAM 握手失败 (用 mock)")
    except Exception as e: print("[hunt] CAM 失败: %s" % e)
  print("[hunt] 就绪. go()=SPIN / see(cx,y2) / cam_test(10)")

# ── 入口 ──
def go(spin_dir=1, hz=20):
  """SPIN 模式 + 自动 run"""
  global _current_sub, _phase_ms, _exit_to, _search_dir, _spin_acc, _rev_start_yaw
  global _confirm_n, _lost_n, _use_mock
  _arb.acquire(_OWNER)
  _current_sub = "SPIN"; _exit_to = ""
  _phase_ms = ticks_ms()
  _search_dir = 1 if spin_dir >= 0 else -1
  _spin_acc = 0.0; _rev_start_yaw = _yaw()
  _confirm_n = 0; _lost_n = 0
  _bearing_pid.reset()
  _use_mock = True  # 默认用 mock；摄像头检测到目标自动切换
  print("[hunt] → SPIN  dir=%d  speed=%.0f" % (_search_dir, _search_speed))
  _run_loop(hz)

def stop():
  _arb.force_brake(); global _current_sub; _current_sub = "IDLE"
  print("[hunt] 停止")

# ── 帧逻辑 ──
def tick():
  global _current_sub, _exit_to, _search_dir, _spin_acc, _rev_start_yaw
  global _confirm_n, _lost_n, _phase_ms
  global _prev_be, _be_ms, _track_ms

  if _current_sub == "IDLE": return
  _poll_camera()
  sensors = _sensors()
  now = ticks_ms()
  has = bool(sensors["has_target"])
  t = sensors["target"]

  # 超时
  if ticks_diff(now, _phase_ms) > _timeout_ms:
    _arb.force_brake()
    _exit_to = "TIMEOUT"; _current_sub = "IDLE"
    print("[hunt] timeout")
    return

  # 确认/丢失计数
  if sensors["new_frame"]:
    if has: _confirm_n += 1
    else: _confirm_n = 0

  # ── SPIN ──
  if _current_sub == "SPIN":
    if sensors["new_frame"] and _confirm_n >= _confirm_frames:
      _current_sub = "TRACK"; _lost_n = 0
      _bearing_pid.reset(); _track_ms = now
      print("[hunt] SPIN → TRACK cls=0 cx=%.0f y2=%.0f" % (_mock_cx, _mock_y2))
      return
    if has:  # 看到但未确认 → 停车等确认
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

  # ── TRACK ──
  if _current_sub == "TRACK":
    y2 = sensors["y2"]
    if sensors["new_frame"]:
      if has and y2 >= _stage_y2:
        _arb.hold_brake(_OWNER)
        if _match_mode != "pre":
          # 检查 yaw 是否已在推箱接近方向
          cls_id = int(t[0]) if t is not None else 0
          push_yaw = _PUSH_HDG.get(cls_id, 0.0)
          approach_yaw = _wrap(push_yaw + 180.0)
          yaw_err = abs(_wrap(_yaw() - approach_yaw))
          if yaw_err <= _ALIGN_YAW_TOL:
            _exit_to = "PUSH (skip ALIGN)"
            print("[hunt] TRACK → PUSH (yaw=%.1f ok, err=%.1f°)" % (_yaw(), yaw_err))
          else:
            _exit_to = "ALIGN"
            print("[hunt] TRACK → ALIGN (yaw=%.1f err=%.1f° > %.0f°)" % (_yaw(), yaw_err, _ALIGN_YAW_TOL))
        else:
          _exit_to = "PUSH"
          print("[hunt] TRACK → PUSH (y2=%.1f)" % y2)
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
  print("[hunt] sub=%s  yaw=%.1f  has=%s  cx=%.1f  y2=%.1f  exit=%s  mock=%s" % (
    _current_sub, _yaw(), _mock_has, _mock_cx, _mock_y2, _exit_to or "-", _use_mock))

def _run_loop(hz=20):
  if _current_sub == "IDLE": print("[run] 请先 go()"); return
  dt = max(10, 1000 // hz); t_last = 0
  print("[run] %d Hz  sub=%s  Ctrl+C 停止" % (hz, _current_sub))
  try:
    while _current_sub not in ("IDLE", "DONE"):
      tick()
      now = ticks_ms()
      if ticks_diff(now, t_last) >= 500:
        t_last = now
        print("  sub=%s  yaw=%+.1f  cx=%.1f  y2=%.1f  has=%s  cf=%d lost=%d mock=%s" % (
          _current_sub, _yaw(), _mock_cx, _mock_y2, _mock_has, _confirm_n, _lost_n, _use_mock))
      sleep_ms(dt)
  except KeyboardInterrupt: pass
  stop()

def run(hz=20):
  _run_loop(hz)

def cam_test(sec=10):
  """摄像头检测 10s"""
  if _cam is None or not _cam.is_ready: print("[cam] 未就绪"); return
  print("[cam] %ds 检测..." % sec)
  t0 = ticks_ms(); n = 0
  try:
    while ticks_diff(ticks_ms(), t0) < sec * 1000:
      frame = _cam.poll()
      if frame is not None and frame.has_target:
        n += 1
        for d in frame.detections:
          print("  cls=%d conf=%d cx=%.0f%% y2=%.0f%%" % (int(d[0]), int(d[1]), d[6], d[9]))
      sleep_ms(50)
  except KeyboardInterrupt: pass
  print("[cam] done: %d frames with target" % n)

init()
