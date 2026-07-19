"""
test_align.py — ALIGN 绕行对位独立测试 (TURN → CLOSE → PUSH)

ALIGN 逻辑:
  TURN:  旋转到推箱接近方向 + 径向保距 + 侧移居中
  CLOSE: 航向OK+居中OK → 最终接近到接触 → PUSH

用法:
  >>> import test_align
  >>> go(90)           # 启动 ALIGN, target_yaw=90°(=推沙包方向)
  >>> see(55, 65)      # 模拟目标 cx=55% y2=65%
  >>> tick()           # 单步
  >>> run(20)          # 20Hz 连续, Ctrl+C 停止
  >>> mon()            # 查看状态
"""

from imu import ImuSensor
from motion import MotionControl, MotorArbiter, HeadingPID, wrap_deg
from time import ticks_ms, ticks_diff, ticks_add, sleep_ms
from smartcar import ticker

# —— 热补丁: 运动学(轮1反号), 等 motion.py 重烧后可删 ——
import math
def _patch_kinematics():
  MotionControl.move_forward = staticmethod(lambda speed: (
    lambda s = float(speed) * MotionControl._FWD_K: [s, -s, 0.0])())
  MotionControl.move_side = staticmethod(lambda speed: (
    lambda s = float(speed) * MotionControl._SIDE_K: [s, s, -2.0 * s])())
  # MicroPython lambda 闭包限制, 用 def 包装
  def _move(speed, angle):
    r = math.radians(-angle)
    c = math.cos(r) / math.sqrt(3)
    s = math.sin(r) / 3
    return [speed*(s+c), speed*(s-c), speed*(-2*s)]
  MotionControl.move = staticmethod(_move)
_patch_kinematics()

# ── 全局 ──
_imu = None; _tkr = None; _motors = None; _arb = None
_OWNER = "ALIGN_TEST"

_yaw_target = 0.0; _phase_ms = 0; _approach_deadline = 0
_ctrl_ms = 0; _orbit_confirm = 0; _vision_lost = 0
_current_sub = "TURN"; _exit_to = ""; _was_yaw_ok = False

_hdg_pid = None; _bearing_pid = None
_rate_yaw = 0.0; _rate_ms = 0; _prev_yaw = 0.0

# 传感器
_mock_cx = 50.0; _mock_y2 = 0.0; _mock_has = False; _mock_new = False
_use_mock = True
_cam = None; _cam_new = False; _cam_last_ms = 0

# 参数
_orbit_speed = 25.0; _orbit_radial_kp = 1.2; _orbit_radial_max = 12.0
_orbit_timeout_ms = 8000; _orbit_yaw_tol_deg = 8.0; _orbit_center_tol_pct = 8.0
_orbit_confirm_frames = 2; _orbit_lost_frames = 5
_orbit_front_spin = 20.0; _orbit_front_slip = 80.0; _orbit_front_flip = False
_approach_speed = 55.0; _final_approach_speed = 45.0; _search_speed = 15.0
_stage_bottom_pct = 85.0; _contact_bottom_pct = 94.0
_confirm_frames = 2; _lost_frames = 2
_bearing_actuation_sign = 1.0; _yaw_actuation_sign = -1.0
_drive_duty = 50.0; _cluster_timeout_ms = 15000

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

def _clamp(v, lo, hi):
  if v < lo: return lo
  if v > hi: return hi
  return v

def _write_spin(duty):
  d = float(duty); _arb.write(_OWNER, [d, d, d])

def _write_vector(forward, lateral, rot):
  fwd = MotionControl.move_forward(float(forward)) if abs(forward) > 1e-6 else (0.0, 0.0, 0.0)
  side = MotionControl.move_side(float(lateral)) if abs(lateral) > 1e-6 else (0.0, 0.0, 0.0)
  duties = [_clamp(fwd[i] + side[i] + rot, -100.0, 100.0) for i in range(3)]
  _arb.write(_OWNER, duties)

def _hold_brake(): _arb.hold_brake(_OWNER)
def _brake(): _arb.force_brake()

# ── 传感器 ──
def see(cx=50.0, y2=60.0):
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
          "target": [0, 31, _mock_cx-10, _mock_y2-10, _mock_cx+10, _mock_y2, _mock_cx, 0, 0, _mock_y2] if _mock_has else None,
          "y2": _mock_y2}

# ── 初始化 ──
def init(use_cam=True):
  global _imu, _tkr, _motors, _arb, _hdg_pid, _bearing_pid, _cam
  print("[align] init IMU963...")
  _imu = ImuSensor(calibrate_samples=200, beta=0.05, model="963")
  _imu._gyro_scale = 1.0
  _tkr = ticker(1); _tkr.capture_list(_imu.raw); _tkr.callback(_tick_imu); _tkr.start(5)
  t0 = ticks_ms()
  while not _imu.is_calibrated:
    sleep_ms(10)
    if ticks_diff(ticks_ms(), t0) > 10000: break
  print("[align] IMU OK yaw=%.2f" % _yaw() if _imu.is_calibrated else "[align] 标定超时")
  _motors = MotionControl(); _arb = MotorArbiter(_motors)
  _hdg_pid = HeadingPID(kp=1.1, max_output=50.0, deadband=1.0, kd=0.08)
  _bearing_pid = HeadingPID(kp=1.2, max_output=60.0, deadband=0.02)
  if use_cam:
    try:
      from machine import UART
      from camera import CameraRx
      _cam = CameraRx(UART(5, baudrate=460800), timeout_ms=5000)
      _cam.flush()
      for i in range(5):
        if _cam.handshake(retries=4, retry_ms=80):
          _cam.set_ready(); print("[align] CAM OK (try %d)" % (i+1)); break
        sleep_ms(100)
      if not _cam.is_ready: print("[align] CAM 握手失败 (用 mock)")
    except Exception as e: print("[align] CAM 失败: %s" % e)
  print("[align] 就绪. go(target_yaw) / see(cx,y2) / run(20)")

# ── 入口 ──
def go(target_yaw, hz=20):
  global _yaw_target, _phase_ms, _approach_deadline, _ctrl_ms, _orbit_confirm, _vision_lost, _current_sub, _use_mock
  _arb.acquire(_OWNER)
  _yaw_target = float(target_yaw); _phase_ms = ticks_ms()
  _approach_deadline = ticks_add(_phase_ms, int(_cluster_timeout_ms))
  _ctrl_ms = _phase_ms; _orbit_confirm = 0; _vision_lost = 0
  _current_sub = "TURN"; _use_mock = True
  _hdg_pid.reset(); _bearing_pid.reset()
  print("[align] → TURN  target_yaw=%.1f  cur=%.1f" % (_yaw_target, _yaw()))
  _run_loop(hz)

def stop():
  _brake(); global _current_sub; _current_sub = "IDLE"
  print("[align] 停止")

# ── 帧逻辑 ──
def tick():
  global _orbit_confirm, _vision_lost, _current_sub, _phase_ms, _was_yaw_ok, _exit_to

  if _current_sub == "IDLE": return
  _poll_camera()
  sensors = _sensors()
  now = ticks_ms()

  if _approach_deadline and ticks_diff(now, _approach_deadline) > 0:
    print("[align] 总超时 → HUNT"); _brake(); _exit_to = "HUNT"; _current_sub = "DONE"; return
  if ticks_diff(now, _phase_ms) > int(_orbit_timeout_ms):
    print("[align] 阶段超时 → HUNT"); _brake(); _exit_to = "HUNT"; _current_sub = "DONE"; return

  target = sensors.get("target") if sensors else None
  if target is None:
    _hold_brake()
    if sensors.get("new_frame"): _vision_lost += 1
    if _vision_lost >= _orbit_lost_frames * 4:
      print("[align] lost long → HUNT"); _brake(); _exit_to = "HUNT"; _current_sub = "DONE"
    elif _vision_lost >= _orbit_lost_frames:
      s = _search_speed * (-1 if (_vision_lost % 2) else 1)
      _write_spin(s)
    return
  if sensors.get("new_frame"): _vision_lost = 0

  cx = float(target[6]); y2 = float(target[9])
  yaw_err = _yaw_err(_yaw_target)
  yaw_tol = float(_orbit_yaw_tol_deg); cx_tol = float(_orbit_center_tol_pct)
  lat_spd = float(_orbit_speed); contact = float(_contact_bottom_pct)
  cx_off = cx - 50.0
  yaw_ok = abs(yaw_err) <= yaw_tol  # 首次进入
  if not yaw_ok and _was_yaw_ok:
    yaw_ok = abs(yaw_err) <= yaw_tol * 1.5  # 滞回: 进来后放宽到1.5×
  _was_yaw_ok = yaw_ok
  cx_ok = abs(cx_off) <= cx_tol
  stage_y2 = float(_stage_bottom_pct)
  radial = _clamp((stage_y2 - y2) * float(_orbit_radial_kp), -float(_orbit_radial_max), float(_orbit_radial_max))

  # ── TURN ──
  if yaw_ok:
    lateral = _clamp((50.0 - cx) / 50.0 * lat_spd, -lat_spd, lat_spd)
    dt = _control_dt(); rot = _yaw_actuation_sign * _hdg_pid.update(yaw_err, dt)
    # 侧移居中时减速前推, 避免冲过头
    cx_abs = abs(cx - 50.0)
    approach = max(radial, 10.0) * max(0.3, 1.0 - cx_abs / 50.0)
    _write_vector(approach, lateral, rot)
    ready = cx_ok and y2 >= contact
    if sensors.get("new_frame"): _orbit_confirm = _orbit_confirm + 1 if ready else 0
    if _orbit_confirm >= int(_orbit_confirm_frames):
      _hold_brake(); _current_sub = "DONE"
      print("[align] → PUSH  yaw=%.1f cx=%.1f y2=%.1f" % (_yaw(), cx, y2))
    return

  # 航向不对 → 绕前方轴转 (move(90°)=LEFT=[-s,s,2s]=[小,小,大])
  edge = min(abs(cx_off) / 50.0, 1.0)
  spin_scale = 1.0 - 0.7 * edge
  dt = _control_dt()
  rate = _yaw_rate()
  rot_n = _yaw_actuation_sign * _hdg_pid.update(yaw_err, dt, rate) * spin_scale / 40.0
  # 最小绕行: |rot_n| < 0.25 时至少给 0.25 防 MIN_DUTY 卡死
  if abs(rot_n) < 0.25: rot_n = 0.25 * (1 if rot_n >= 0 else -1)
  rot_n = _clamp(rot_n, -1.0, 1.0)
  spin = rot_n * _orbit_front_spin; slip = rot_n * _orbit_front_slip
  if _orbit_front_flip: slip = -slip
  lat_extra = _clamp((50.0 - cx) / 50.0 * lat_spd * 0.4, -lat_spd * 0.4, lat_spd * 0.4)
  side_total = slip + lat_extra
  side = MotionControl.move(side_total, 90.0) if abs(side_total) > 1e-6 else (0.0, 0.0, 0.0)
  # 绕行时不前推，保持距离；由 yaw_ok 路径的 radial 负责保距
  fwd = (0.0, 0.0, 0.0)
  duties = [_clamp(fwd[i] + side[i] + spin, -100.0, 100.0) for i in range(3)]
  _arb.write(_OWNER, duties)
  if sensors.get("new_frame"): _orbit_confirm = 0

# ── 监控 ──
def mon():
  print("[align] sub=%s  yaw=%.1f  target=%.1f  err=%.1f°  cx=%.1f  y2=%.1f  has=%s  ok=%d lost=%d mock=%s" % (
    _current_sub, _yaw(), _yaw_target, _yaw_err(_yaw_target),
    _mock_cx, _mock_y2, _mock_has, _orbit_confirm, _vision_lost, _use_mock))

def _run_loop(hz=20):
  if _current_sub == "IDLE": print("[run] 请先 go(target_yaw)"); return
  dt = max(10, 1000 // hz); t_last = 0
  print("[run] %d Hz  sub=%s  Ctrl+C 停止" % (hz, _current_sub))
  try:
    while _current_sub not in ("IDLE", "DONE"):
      tick()
      now = ticks_ms()
      if ticks_diff(now, t_last) >= 300:
        t_last = now
        print("  sub=%s  yaw=%+.1f  err=%+.1f°  cx=%.1f  y2=%.1f  ok=%d" % (
          _current_sub, _yaw(), _yaw_err(_yaw_target), _mock_cx, _mock_y2, _orbit_confirm))
      sleep_ms(dt)
  except KeyboardInterrupt: pass
  stop()

def run(hz=20): _run_loop(hz)

init()
