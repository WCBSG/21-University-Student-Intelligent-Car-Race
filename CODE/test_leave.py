"""
test_leave.py — LEAVE 出库状态独立测试 (EXIT → SHIFT)

两阶段:
  EXIT:  直行锁航向，直到离线（离开黄线）
  SHIFT: 根据发车位置横向平移，同样锁航向，超时→HUNT

用法:
  >>> import test_leave
  >>> go(layout=2, duty=50)   # 发车位置: 0/1=中 2=左下 3=右下 4=左中
  >>> tick()                  # 单步
  >>> run(20)                 # 20Hz 连续, Ctrl+C 停止
  >>> mon()                   # 查看状态
  >>> spin_test(30)           # 三轮同速自旋
  >>> fwd_test(30)            # move_forward 验证
  >>> wheel_test(0, 30)       # 单轮测试
"""

from imu import ImuSensor
from motion import MotionControl, MotorArbiter, HeadingPID, wrap_deg

# —— 热补丁: 运动学(轮1反号), 等 motion.py 重烧后可删 ——
import math
_MC_FWD_SIDE_PATCHED = False
def _patch_kinematics():
  global _MC_FWD_SIDE_PATCHED
  if _MC_FWD_SIDE_PATCHED:
    return
  MotionControl.move_forward = staticmethod(lambda speed: (
    lambda s = float(speed) * MotionControl._FWD_K: [s, -s, 0.0])())
  MotionControl.move_side = staticmethod(lambda speed: (
    lambda s = float(speed) * MotionControl._SIDE_K: [s, s, -2.0 * s])())
  def _move(speed, angle):
    r = math.radians(-angle)
    c = math.cos(r) / math.sqrt(3)
    s = math.sin(r) / 3
    return [speed*(s+c), speed*(s-c), speed*(-2*s)]
  MotionControl.move = staticmethod(_move)
  _MC_FWD_SIDE_PATCHED = True
  print("[patch] move/move_forward/move_side 已修正")
from time import ticks_ms, ticks_diff, sleep_ms
from smartcar import ticker

# ── 全局 ──
_imu = None; _tkr = None; _tkr_tcs = None
_motors = None; _arb = None
_OWNER = "LEAVE_TEST"

# 状态
_hold_yaw = 0.0; _drive_duty = 50.0; _shift_duty = 40.0
_phase_ms = 0; _exit_timeout_ms = 4000; _shift_timeout_ms = 3000
_current_sub = "IDLE"  # EXIT | SHIFT | DONE
_exit_to = ""
_layout = 0  # 发车位置
_shift_dir = 0  # +1=右移, -1=左移, 0=直行
_saw_line = False  # EXIT 阶段是否已压过黄线

# PID
_hdg_pid = None; _hdg_ms = 0

# 传感器
_tcs = None; _tcs_on_line = False; _tcs_ready = False
_cam = None
_cam_has = False; _cam_target = None; _cam_y2 = 0.0
_cam_new = False; _cam_last_ms = 0; _cam_timeout = False

# IMU ticks
_tick_n = 0; _imu_up = False
_rate_yaw = 0.0; _rate_ms = 0; _prev_yaw = 0.0

def _tick_imu(_):
  global _tick_n, _imu_up
  try: _imu.update(); _tick_n += 1; _imu_up = True
  except Exception: pass

def _tick_tcs(_):
  global _tcs_on_line, _imu_up
  if _tcs_ready:
    try:
      _tcs.crossed_yellow()
      _tcs_on_line = _tcs.on_line
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
  global _hdg_ms
  now = ticks_ms(); dt = ticks_diff(now, _hdg_ms) / 1000.0
  if dt <= 0.0 or dt > 0.5: dt = 0.02
  _hdg_ms = now; return dt

def _clamp(v, lo, hi):
  if v < lo: return lo
  if v > hi: return hi
  return v

def _pct_to_pwm(pct):
  return int((100 - max(0, min(100, pct))) * 65535 / 100)

def _pwm_regs():
  try:
    return [(int(_motors._motors[i][0].duty_u16()),
             int(_motors._motors[i][1].duty_u16())) for i in range(3)]
  except Exception:
    return [(0,0),(0,0),(0,0)]

def pwm_dump():
  regs = _pwm_regs()
  for i, (ccw, cw) in enumerate(regs):
    print("  M%d  CCW=%5d (%.0f%%)  CW=%5d (%.0f%%)" % (
      i, ccw, (65535-ccw)/65535*100, cw, (65535-cw)/65535*100))

# ── 传感器读取 ──
def _pick_best_target(detections):
  """从 detections 中选最佳目标（免 config 依赖）"""
  best = None; best_area = 0
  for d in detections:
    cls_id = int(d[0])     # camera._parse 已解码: d[0]=cls, d[1]=conf
    conf = int(d[1])
    if conf < 2:  # 最低置信度 (0-31)
      continue
    area = d[8] if len(d) > 8 else (d[3]*d[4])
    if area > best_area:
      best_area = area; best = d
  return best

def _poll_camera():
  global _cam_has, _cam_target, _cam_y2, _cam_new, _cam_last_ms, _cam_timeout
  _cam_new = False
  if _cam is None or not _cam.is_ready:
    return
  frame = _cam.poll()
  if frame is not None:
    _cam_last_ms = ticks_ms()
    _cam_new = True
    _cam_has = frame.has_target
    if _cam_has and frame.detections:
      _cam_target = _pick_best_target(frame.detections)
      if _cam_target is not None:
        _cam_y2 = _cam_target[9]
      else:
        _cam_has = False; _cam_y2 = 0.0
    else:
      _cam_target = None; _cam_y2 = 0.0
  elif _cam_last_ms and ticks_diff(ticks_ms(), _cam_last_ms) > 300:
    _cam_has = False; _cam_target = None; _cam_y2 = 0.0
  _cam_timeout = _cam.timed_out if _cam else False

def _sensors():
  return {
    "new_frame": _cam_new,
    "has_target": _cam_has,
    "target": _cam_target,
    "y2": _cam_y2,
    "tcs_on_line": _tcs_on_line,
  }

def _on_line():
  return _tcs_on_line

# ── 初始化 ──
def init(use_cam=True, use_tcs=True):
  global _imu, _tkr, _tkr_tcs, _motors, _arb, _hdg_pid
  global _tcs, _tcs_ready, _cam

  print("[leave] init IMU963...")
  _patch_kinematics()
  _imu = ImuSensor(calibrate_samples=200, beta=0.05, model="963")
  _imu._gyro_scale = 1.0
  _tkr = ticker(1); _tkr.capture_list(_imu.raw); _tkr.callback(_tick_imu); _tkr.start(5)
  t0 = ticks_ms()
  while not _imu.is_calibrated:
    sleep_ms(10)
    if ticks_diff(ticks_ms(), t0) > 10000: break
  print("[leave] IMU OK yaw=%.2f" % _yaw() if _imu.is_calibrated else "[leave] IMU 标定超时")

  # TCS (黄线)
  if use_tcs:
    try:
      from tcs3472 import TCS3472, make_i2c
      _tcs = TCS3472(make_i2c())
      _tcs.confirm_n = 2
      _tcs_ready = True
      _tkr_tcs = ticker(2); _tkr_tcs.callback(_tick_tcs); _tkr_tcs.start(20)
      print("[leave] TCS OK")
    except Exception as e:
      print("[leave] TCS 失败: %s" % e)

  # Camera
  if use_cam:
    try:
      from machine import UART
      from camera import CameraRx
      _cam = CameraRx(UART(5, baudrate=460800), timeout_ms=5000)
      # 握手
      _cam.flush()
      for i in range(5):
        if _cam.handshake(retries=4, retry_ms=80):
          _cam.set_ready()
          print("[leave] CAM OK (try %d)" % (i+1))
          break
        sleep_ms(100)
      if not _cam.is_ready:
        print("[leave] CAM 握手失败 (继续无视觉)")
    except Exception as e:
      print("[leave] CAM 失败: %s" % e)

  _motors = MotionControl(); _arb = MotorArbiter(_motors)
  _hdg_pid = HeadingPID(kp=1.1, max_output=50.0, deadband=1.0, kd=0.08)
  print("[leave] 就绪. go(layout=2, duty=50)")

# ── 控制 ──
def _shift_for_layout(layout):
  """返回 (shift_dir, label): +1=右移, -1=左移, 0=直行"""
  layout = int(layout)
  if layout in (0, 1):
    return 0, "STRAIGHT"
  if layout == 2:  # 左下角
    return 1, "RIGHT"
  if layout == 3:  # 右下角
    return -1, "LEFT"
  if layout == 4:  # 左边中
    return 1, "RIGHT"
  return 0, "STRAIGHT"

def go(layout=2, duty=50, exit_timeout_ms=4000, shift_ms=3000, hz=20):
  global _layout, _hold_yaw, _drive_duty, _phase_ms, _current_sub, _exit_to
  global _exit_timeout_ms, _shift_timeout_ms, _shift_dir, _saw_line
  _arb.acquire(_OWNER)
  _layout = int(layout); _drive_duty = float(duty)
  _hold_yaw = _yaw()
  _exit_timeout_ms = int(exit_timeout_ms)
  _shift_timeout_ms = int(shift_ms)
  _shift_dir, label = _shift_for_layout(_layout)
  _current_sub = "EXIT"; _exit_to = ""; _saw_line = False
  _phase_ms = ticks_ms()
  _hdg_pid.reset()
  print("[leave] → EXIT  layout=%d(%s)  hold_yaw=%.1f  duty=%.0f  exit_timeout=%dms  shift=%dms dir=%s" % (
    _layout, ["中","中","左下","右下","左中"][min(_layout,4)], _hold_yaw, _drive_duty, _exit_timeout_ms, _shift_timeout_ms, label))
  # 直接开跑
  _run_loop(hz)

def tick():
  global _current_sub, _exit_to, _phase_ms, _shift_dir
  if _current_sub == "IDLE": return

  _poll_camera()
  sensors = _sensors()
  now = ticks_ms()

  # 看到目标 → 提前退出
  if sensors["new_frame"] and sensors["has_target"] and sensors["target"] is not None:
    t = sensors["target"]
    cls_id = int(t[0])
    y2 = float(t[9])
    print("[leave] see target cls=%d y2=%.1f → HUNT/ALIGN" % (cls_id, y2))
    _arb.force_brake()
    _exit_to = "HUNT (see target)"
    _current_sub = "IDLE"
    return

  # ── EXIT: 直行 → 压黄线 → 离线 → SHIFT ──
  if _current_sub == "EXIT":
    global _saw_line
    if ticks_diff(now, _phase_ms) > _exit_timeout_ms:
      # 超时也进 SHIFT
      _current_sub = "SHIFT"; _phase_ms = now
      if _shift_dir == 0:
        _arb.force_brake()
        _exit_to = "HUNT (exit timeout, straight)"
        _current_sub = "IDLE"
        print("[leave] EXIT 超时 → %s" % _exit_to)
      else:
        print("[leave] EXIT 超时 → SHIFT dir=%+d" % _shift_dir)
      return

    if _on_line():
      _saw_line = True  # 压到黄线

    if _saw_line and not _on_line():
      # 压过黄线后离线 → 进 SHIFT
      _current_sub = "SHIFT"; _phase_ms = now
      if _shift_dir == 0:
        _arb.force_brake()
        _exit_to = "HUNT (crossed line, straight)"
        _current_sub = "IDLE"
        print("[leave] EXIT 跨线 → %s" % _exit_to)
      else:
        print("[leave] EXIT 跨线 → SHIFT dir=%+d" % _shift_dir)
      return

    # 直行 + 航向锁
    _write_forward_locked(_drive_duty, _hold_yaw)
    return

  # ── SHIFT: 横向平移 ──
  if _current_sub == "SHIFT":
    if ticks_diff(now, _phase_ms) > _shift_timeout_ms:
      _arb.force_brake()
      _exit_to = "HUNT (shift done)"
      _current_sub = "IDLE"
      print("[leave] SHIFT 完成 → %s" % _exit_to)
      return

    _write_lateral_locked(_shift_duty, _hold_yaw, _shift_dir)
    return

def _write_forward_locked(speed, yaw_tgt):
  dt = _control_dt()
  err = _yaw_err(yaw_tgt)
  rate = _yaw_rate()
  rot = -1.0 * _hdg_pid.update(err, dt, rate)  # yaw_actuation_sign=-1
  fwd = MotionControl.move_forward(float(speed))
  duties = [_clamp(fwd[i] + rot, -100.0, 100.0) for i in range(3)]
  _arb.write(_OWNER, duties)

def _write_lateral_locked(speed, yaw_tgt, shift_dir):
  dt = _control_dt()
  err = _yaw_err(yaw_tgt)
  rate = _yaw_rate()
  rot = -1.0 * _hdg_pid.update(err, dt, rate)
  lat = float(speed) * float(shift_dir)  # >0 右移, <0 左移
  side = MotionControl.move_side(lat) if abs(lat) > 1e-6 else (0.0, 0.0, 0.0)
  duties = [_clamp(side[i] + rot, -100.0, 100.0) for i in range(3)]
  _arb.write(_OWNER, duties)

def stop():
  global _current_sub
  _arb.force_brake(); _current_sub = "IDLE"
  print("[leave] 停止")

def mon():
  if _current_sub == "IDLE":
    print("[leave] IDLE  exit=%s" % (_exit_to or "-"))
  else:
    print("[leave] sub=%s  yaw=%.1f  hold=%.1f  err=%.1f°  online=%s  cam=%s/%s  exit=%s" % (
      _current_sub, _yaw(), _hold_yaw, _yaw_err(_hold_yaw),
      _on_line(), _cam_has, "new" if _cam_new else "-",
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
        print("  sub=%s  yaw=%+.1f  err=%+.1f°  online=%s  cam=%s" % (
          _current_sub, _yaw(), _yaw_err(_hold_yaw), _on_line(), _cam_has))
      sleep_ms(dt)
  except KeyboardInterrupt:
    stop()

# 兼容旧调用
def run(hz=20):
  _run_loop(hz)

# ── 电机极性验证 ──
def spin_test(duty=30, ms=500):
  _arb.acquire(_OWNER)
  y0 = _yaw()
  _arb.write(_OWNER, [duty, duty, duty])
  print("[spin] duty=%+d%% [%d,%d,%d]" % (duty, duty, duty, duty))
  pwm_dump()
  sleep_ms(ms)
  _arb.force_brake()
  y1 = _yaw()
  dy = wrap_deg(y1 - y0)
  direction = "CCW(yaw↑)" if dy > 0 else "CW(yaw↓)"
  print("  yaw %.1f→%.1f  Δ=%.1f°  %s" % (y0, y1, dy, direction))

def fwd_test(duty=30, ms=2000):
  _arb.acquire(_OWNER)
  y0 = _yaw()
  d = MotionControl.move_forward(duty)
  _arb.write(_OWNER, d)
  print("[fwd] duty=%+d%%  →  duties=%s" % (duty, [round(x,1) for x in d]))
  pwm_dump()
  sleep_ms(ms)
  _arb.force_brake()
  y1 = _yaw()
  dy = wrap_deg(y1 - y0)
  print("  yaw %.1f→%.1f  Δ=%.1f°  (看车朝哪走)" % (y0, y1, dy))

def side_test(duty=30, ms=2000):
  """move_side 验证：看车往哪走。duty>0 应右移"""
  _arb.acquire(_OWNER)
  y0 = _yaw()
  d = MotionControl.move_side(duty)
  _arb.write(_OWNER, d)
  print("[side] duty=%+d%%  →  duties=%s" % (duty, [round(x,1) for x in d]))
  pwm_dump()
  sleep_ms(ms)
  _arb.force_brake()
  y1 = _yaw()
  dy = wrap_deg(y1 - y0)
  print("  yaw %.1f→%.1f  Δ=%.1f°  (看车往哪走)" % (y0, y1, dy))

def wheel_test(i, duty=30, ms=300):
  duties = [0.0, 0.0, 0.0]; duties[i] = float(duty)
  _arb.acquire(_OWNER)
  y0 = _yaw()
  _arb.write(_OWNER, duties)
  print("[wheel%d] duty=%+d%%  →  duties=%s" % (i, duty, [round(d,1) for d in duties]))
  pwm_dump()
  sleep_ms(ms)
  _arb.force_brake()
  y1 = _yaw()
  dy = wrap_deg(y1 - y0)
  print("  yaw %.1f→%.1f  Δ=%.1f°  (看车往哪走)" % (y0, y1, dy))

def cam_test(sec=10):
  """摄像头识别测试：连续 polling 打印检测结果"""
  if _cam is None or not _cam.is_ready:
    print("[cam] 摄像头未就绪"); return
  print("[cam] 开始 %ds 检测 (Ctrl+C 停止)..." % sec)
  t0 = ticks_ms(); n_frame = 0; n_det = 0
  try:
    while ticks_diff(ticks_ms(), t0) < sec * 1000:
      frame = _cam.poll()
      if frame is not None:
        n_frame += 1
        if frame.has_target:
          n_det += 1
          for d in frame.detections:
            cls_id = int(d[0])     # camera._parse 已解码
            conf = int(d[1])
            cx = d[6] if len(d) > 6 else 0
            y2 = d[9] if len(d) > 9 else 0
            print("  cls=%d conf=%d cx=%.0f%% y2=%.0f%%" % (cls_id, conf, cx, y2))
      sleep_ms(50)
  except KeyboardInterrupt: pass
  dt = max(1, ticks_diff(ticks_ms(), t0) / 1000.0)
  print("[cam] done: %d frames, %d with target (%.1f FPS)" % (n_frame, n_det, n_frame / dt))

init()
