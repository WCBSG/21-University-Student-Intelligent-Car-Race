"""
align_test.py — ALIGN 状态机独立测试工具

用法:
  >>> import align_test
  >>> go(90)           # 启动 ALIGN, target_yaw=90°
  >>> see(55, 65)      # 模拟看到目标 cx=55% y2=65%
  >>> see(48, 72)      # 更新目标位置
  >>> tick()           # 单步一帧
  >>> run(20)          # 20Hz 连续运行, Ctrl+C 停止
  >>> hide()           # 目标丢失
  >>> mon()            # 查看状态
  >>> stop()           # 刹车停止
  >>> cfg()            # 打印相关参数
"""

from seekfree import IMU963RX
from smartcar import ticker
from imu import ImuSensor, MadgwickAHRS
from imu import _acc_to_g, _gyro_to_radps, GYRO_LSB_963, ACC_LSB_PER_G
from motion import MotionControl, MotorArbiter, HeadingPID, wrap_deg
from time import ticks_ms, ticks_diff, ticks_add, sleep_ms

# ============================================================
# 全局状态
# ============================================================

_imu = None
_tkr = None
_motors = None
_arb = None
_OWNER = "ALIGN_TEST"

# ALIGN 状态
_yaw_target = 0.0
_phase_ms = 0
_approach_deadline = 0
_ctrl_ms = 0
_orbit_confirm = 0
_vision_lost = 0
_current_sub = "TURN"

# PID
_hdg_pid = None
_bearing_pid = None

# 模拟传感器
_mock_cx = 50.0
_mock_y2 = 0.0
_mock_has_target = False
_mock_new_frame = False

# 配置默认值
_orbit_speed = 40.0
_orbit_radial_kp = 1.2
_orbit_radial_max = 12.0
_orbit_timeout_ms = 8000
_orbit_yaw_tol_deg = 8.0
_orbit_center_tol_pct = 8.0
_orbit_confirm_frames = 2
_orbit_lost_frames = 3
_orbit_front_spin = 20.0
_orbit_front_slip = 80.0
_orbit_front_flip = False
_approach_speed = 55.0
_final_approach_speed = 45.0
_search_speed = 15.0
_stage_bottom_pct = 75.0
_contact_bottom_pct = 94.0
_confirm_frames = 2
_lost_frames = 2
_bearing_actuation_sign = 1.0
_yaw_actuation_sign = -1.0
_drive_duty = 50.0
_cluster_timeout_ms = 15000
_heading_kp = 1.1
_heading_max = 50.0
_heading_deadband = 1.0

_tick_n = 0


def _tick(_):
  global _tick_n
  try:
    _imu.update()
    _tick_n += 1
  except Exception:
    pass


# ============================================================
# 初始化
# ============================================================

def init():
  """初始化 IMU + 电机。"""
  global _imu, _tkr, _motors, _arb, _hdg_pid, _bearing_pid

  print("[align] init IMU963...")
  _imu = ImuSensor(calibrate_samples=200, beta=0.05, model="963")
  _imu._gyro_scale = 1.0
  _imu._spin_beta = 0.01
  _imu._spin_dps = 40.0

  _tkr = ticker(1)
  _tkr.capture_list(_imu.raw)
  _tkr.callback(_tick)
  _tkr.start(5)

  print("[align] 标定中...")
  t0 = ticks_ms()
  while not _imu.is_calibrated:
    sleep_ms(10)
    if ticks_diff(ticks_ms(), t0) > 10000:
      print("[align] 标定超时!")
      break
  if _imu.is_calibrated:
    print("[align] OK yaw=%.2f" % _imu.get_yaw())

  _motors = MotionControl()
  _arb = MotorArbiter(_motors)
  _hdg_pid = HeadingPID(kp=_heading_kp, max_output=_heading_max,
                         deadband=_heading_deadband)
  _bearing_pid = HeadingPID(kp=1.2, max_output=60.0, deadband=0.02)

  print("[align] 就绪. go(90) 启动 / see(cx,y2) 喂目标 / run(20) 连续 / mon() 查看")


# ============================================================
# 模拟传感器
# ============================================================

def see(cx=50.0, y2=60.0):
  """喂模拟目标: cx=水平居中%(50=正中), y2=底部Y%"""
  global _mock_cx, _mock_y2, _mock_has_target, _mock_new_frame
  _mock_cx = float(cx)
  _mock_y2 = float(y2)
  _mock_has_target = True
  _mock_new_frame = True


def hide():
  """目标丢失。"""
  global _mock_has_target, _mock_new_frame
  _mock_has_target = False
  _mock_new_frame = True


def _mock_sensors():
  """构造模拟传感器字典。"""
  global _mock_new_frame
  nf = _mock_new_frame
  _mock_new_frame = False
  cx = _mock_cx
  y2 = _mock_y2
  return {
    "new_frame": nf,
    "has_target": _mock_has_target,
    "target": [0, 31, cx - 10, y2 - 10, cx + 10, y2, cx, 0, 0, y2] if _mock_has_target else None,
    "y2": y2,
  }


# ============================================================
# 辅助
# ============================================================

def _yaw():
  return _imu.get_yaw()


def _yaw_err(target):
  return wrap_deg(target - _yaw())


def _control_dt():
  global _ctrl_ms
  now = ticks_ms()
  dt = ticks_diff(now, _ctrl_ms) / 1000.0
  if dt <= 0.0 or dt > 0.5:
    dt = 0.05
  _ctrl_ms = now
  return dt


def _clamp(v, lo, hi):
  if v < lo: return lo
  if v > hi: return hi
  return v


def _write_spin(duty):
  d = float(duty)
  _arb.write(_OWNER, [d, d, d])


def _write_vector(forward, lateral, rot):
  fwd = MotionControl.move(float(forward), 0.0)
  if lateral >= 0.0:
    side = MotionControl.move(float(lateral), 90.0)
  else:
    side = MotionControl.move(float(-lateral), -90.0)
  duties = [_clamp(fwd[i] + side[i] + rot, -100.0, 100.0) for i in range(3)]
  _arb.write(_OWNER, duties)


def _write_orbit_front(spin, slip, forward=0.0):
  spin = float(spin)
  slip = float(slip)
  if slip >= 0.0:
    side = MotionControl.move(slip, 90.0)
  else:
    side = MotionControl.move(-slip, -90.0)
  if abs(forward) > 1e-6:
    fwd = MotionControl.move(float(forward), 0.0)
  else:
    fwd = (0.0, 0.0, 0.0)
  duties = [_clamp(fwd[i] + side[i] + spin, -100.0, 100.0) for i in range(3)]
  _arb.write(_OWNER, duties)


def _hold_brake():
  _arb.hold_brake(_OWNER)


def _brake():
  _arb.force_brake()


# ============================================================
# ALIGN 控制
# ============================================================

def go(target_yaw):
  """启动 ALIGN: 设置推箱目标航向。"""
  global _yaw_target, _phase_ms, _approach_deadline, _ctrl_ms
  global _orbit_confirm, _vision_lost, _current_sub
  _arb.acquire(_OWNER)
  _yaw_target = float(target_yaw)
  _phase_ms = ticks_ms()
  _approach_deadline = ticks_add(_phase_ms, int(_cluster_timeout_ms))
  _ctrl_ms = _phase_ms
  _orbit_confirm = 0
  _vision_lost = 0
  _current_sub = "TURN"
  _hdg_pid.reset()
  _bearing_pid.reset()
  print("[align] → TURN  target_yaw=%.1f  cur=%.1f" % (_yaw_target, _yaw()))


def stop():
  """刹车停止。"""
  _brake()
  print("[align] 刹车停止")


def _align_lost_soft(sensors):
  """丢目标处理。返回 True=已退出 ALIGN。"""
  global _vision_lost, _current_sub
  _hold_brake()
  if sensors and sensors.get("new_frame"):
    _vision_lost += 1
  wait = int(_orbit_lost_frames)
  if _vision_lost < wait:
    return False
  # 中丢：反转找
  if _vision_lost == wait:
    print("[align] TURN lost → reverse find")
  if _vision_lost < wait * 4:
    s = float(_search_speed) * (-1 if _vision_lost % 2 == 0 else 1)
    _write_spin(s)
    return False
  print("[align] TURN lost long → abort")
  _brake()
  return True


def tick():
  """执行一帧 ALIGN 逻辑。"""
  global _orbit_confirm, _vision_lost, _current_sub, _phase_ms

  if _current_sub == "IDLE":
    return

  now = ticks_ms()
  sensors = _mock_sensors()

  # 超时检查
  if _approach_deadline and ticks_diff(now, _approach_deadline) > 0:
    print("[align] 总超时 → 退出")
    _brake()
    _current_sub = "IDLE"
    return
  if ticks_diff(now, _phase_ms) > int(_orbit_timeout_ms):
    print("[align] 阶段超时 → 退出")
    _brake()
    _current_sub = "IDLE"
    return

  target = sensors.get("target") if sensors else None
  if target is None:
    if _align_lost_soft(sensors):
      _current_sub = "IDLE"
    return
  if sensors and sensors.get("new_frame"):
    _vision_lost = 0

  cx = float(target[6])
  y2 = float(target[9])
  yaw_err = _yaw_err(_yaw_target)
  yaw_tol = float(_orbit_yaw_tol_deg)
  cx_tol = float(_orbit_center_tol_pct)
  lat_spd = float(_orbit_speed)
  contact = float(_contact_bottom_pct)
  cx_off = cx - 50.0
  yaw_ok = abs(yaw_err) <= yaw_tol
  cx_ok = abs(cx_off) <= cx_tol
  stage_y2 = float(_stage_bottom_pct)
  radial = (stage_y2 - y2) * float(_orbit_radial_kp)
  radial = _clamp(radial, -float(_orbit_radial_max), float(_orbit_radial_max))

  # ── CLOSE ──
  if _current_sub == "CLOSE":
    if abs(yaw_err) > yaw_tol * 1.5:
      _current_sub = "TURN"
      _orbit_confirm = 0
      _hdg_pid.reset()
      print("[align] CLOSE → TURN (yaw=%.1f err=%.1f)" % (_yaw(), yaw_err))
      return
    if y2 >= contact:
      if 30.0 <= cx <= 78.0:
        _hold_brake()
        print("[align] CLOSE → PUSH (接触)  cx=%.1f y2=%.1f" % (cx, y2))
        _current_sub = "DONE"
      else:
        lateral = _clamp((50.0 - cx) / 50.0 * lat_spd, -lat_spd, lat_spd)
        dt = _control_dt()
        rot = float(_yaw_actuation_sign) * _hdg_pid.update(yaw_err, dt)
        _write_vector(0.0, lateral, rot)
      return
    lateral = _clamp((50.0 - cx) / 50.0 * lat_spd, -lat_spd, lat_spd)
    dt = _control_dt()
    rot = float(_yaw_actuation_sign) * _hdg_pid.update(yaw_err, dt) * 0.35
    _write_vector(float(_final_approach_speed), lateral, rot)
    return

  # ── TURN ──
  if yaw_ok:
    lateral = _clamp((50.0 - cx) / 50.0 * lat_spd, -lat_spd, lat_spd)
    dt = _control_dt()
    rot = float(_yaw_actuation_sign) * _hdg_pid.update(yaw_err, dt) * 0.4
    _write_vector(radial, lateral, rot)
    ready = cx_ok
    if sensors and sensors.get("new_frame"):
      _orbit_confirm = _orbit_confirm + 1 if ready else 0
    if _orbit_confirm >= int(_orbit_confirm_frames):
      _current_sub = "CLOSE"
      _bearing_pid.reset()
      _hdg_pid.reset()
      print("[align] TURN → CLOSE  cx=%.1f yaw=%.1f" % (cx, _yaw()))
    return

  # 航向不对：绕前方轴转
  edge = abs(cx_off) / 50.0
  if edge > 1.0: edge = 1.0
  spin_scale = 1.0 - 0.7 * edge
  dt = _control_dt()
  rot_n = float(_yaw_actuation_sign) * _hdg_pid.update(yaw_err, dt) * spin_scale
  rot_n = _clamp(rot_n / 40.0, -1.0, 1.0)
  spin = rot_n * float(_orbit_front_spin)
  slip = rot_n * float(_orbit_front_slip)
  if bool(_orbit_front_flip):
    slip = -slip
  lat_extra = _clamp((50.0 - cx) / 50.0 * lat_spd * 0.4, -lat_spd * 0.4, lat_spd * 0.4)
  _write_orbit_front(spin, slip + lat_extra, radial)
  if sensors and sensors.get("new_frame"):
    _orbit_confirm = 0


# ============================================================
# 监控 & 连续运行
# ============================================================

def mon():
  """打印当前 ALIGN 状态 + 传感器。"""
  yaw = _yaw()
  err = _yaw_err(_yaw_target)
  print("[align] sub=%s  yaw=%.1f  target=%.1f  err=%.1f°" % (
    _current_sub, yaw, _yaw_target, err))
  print("        cx=%.1f  y2=%.1f  has_tgt=%s  orbit_ok=%d  lost=%d" % (
    _mock_cx, _mock_y2, _mock_has_target, _orbit_confirm, _vision_lost))


def run(hz=20):
  """连续运行 ALIGN 帧, Ctrl+C 停止。调用前先 see() 喂目标。"""
  if _current_sub == "IDLE":
    print("[run] 请先 go(target_yaw) 和 see(cx, y2)")
    return
  dt = max(10, 1000 // hz)
  t_last = 0
  print("[run] %d Hz  sub=%s  Ctrl+C 停止" % (hz, _current_sub))
  try:
    while True:
      tick()
      now = ticks_ms()
      if ticks_diff(now, t_last) >= 300:
        t_last = now
        yaw = _yaw()
        err = _yaw_err(_yaw_target)
        print("  sub=%s  yaw=%+.1f  err=%+.1f°  cx=%.1f  y2=%.1f  confirm=%d" % (
          _current_sub, yaw, err, _mock_cx, _mock_y2, _orbit_confirm))
      sleep_ms(dt)
  except KeyboardInterrupt:
    pass
  _brake()
  print("[run] stop")


# ============================================================
# 参数
# ============================================================

def cfg():
  """打印 ALIGN 相关参数。"""
  ks = [
    ("orbit_speed", _orbit_speed),
    ("orbit_radial_kp", _orbit_radial_kp),
    ("orbit_radial_max", _orbit_radial_max),
    ("orbit_timeout_ms", _orbit_timeout_ms),
    ("orbit_yaw_tol_deg", _orbit_yaw_tol_deg),
    ("orbit_center_tol_pct", _orbit_center_tol_pct),
    ("orbit_confirm_frames", _orbit_confirm_frames),
    ("orbit_lost_frames", _orbit_lost_frames),
    ("orbit_front_spin", _orbit_front_spin),
    ("orbit_front_slip", _orbit_front_slip),
    ("orbit_front_flip", _orbit_front_flip),
    ("approach_speed", _approach_speed),
    ("final_approach_speed", _final_approach_speed),
    ("search_speed", _search_speed),
    ("stage_bottom_pct", _stage_bottom_pct),
    ("contact_bottom_pct", _contact_bottom_pct),
    ("confirm_frames", _confirm_frames),
    ("lost_frames", _lost_frames),
    ("bearing_actuation_sign", _bearing_actuation_sign),
    ("yaw_actuation_sign", _yaw_actuation_sign),
    ("heading_kp", _heading_kp),
    ("heading_max", _heading_max),
    ("heading_deadband", _heading_deadband),
    ("cluster_timeout_ms", _cluster_timeout_ms),
  ]
  print("[cfg] ─── ALIGN 参数 ───")
  for k, v in ks:
    print("  %-24s = %s" % (k, v))


def set_cfg(k, v):
  """改参数: set_cfg('orbit_speed', 30)。"""
  g = globals()
  key = "_" + k if not k.startswith("_") else k
  if key not in g and k in g:
    key = k
  if key not in g:
    print("[set_cfg] 未知: %s" % k)
    return
  old = g[key]
  try:
    t = type(old)
    if t is bool: v2 = str(v).lower() in ("1", "true", "yes", "on")
    elif t is int: v2 = int(float(v))
    else: v2 = t(float(v))
    g[key] = v2
    print("[set_cfg] %s: %s → %s" % (k, old, v2))
  except Exception:
    print("[set_cfg] 类型错误")


# 自动初始化
init()
