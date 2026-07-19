"""
test_otto.py — MotionControlOtto 运动学对比测试

用法:
  >>> import test_otto
  >>> fwd(30)    # 前向
  >>> side(30)   # 右移
  >>> spin(30)   # 自旋
  >>> orbit(25, -15)  # 右绕轴转 (speed, spin)
  >>> set_map(0,1,2)  # 调整轮序映射
"""

from imu import ImuSensor
from motion import MotionControl, MotorArbiter, MotionControlOtto, wrap_deg
from time import ticks_ms, ticks_diff, sleep_ms
from smartcar import ticker
import math as _math

# —— 热补丁（不覆盖 move/move_forward/move_side，保留原始） ——
def _patch_kinematics():
  """修正原始 MotionControl 的 move_forward/move_side（手写版有 bug）"""
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
_OWNER = "OTTO_TEST"
_ctrl_ms = 0; _tick_n = 0
_pwm_cache = [0, 0, 0]

def _tick_imu(_):
  global _tick_n
  try: _imu.update(); _tick_n += 1
  except Exception: pass

def _yaw(): return _imu.get_yaw() if _imu and _imu.is_calibrated else 0.0

# ── 初始化 ──
def init():
  global _imu, _tkr, _motors, _arb
  print("[otto] init IMU963...")
  _imu = ImuSensor(calibrate_samples=200, beta=0.05, model="963")
  _imu._gyro_scale = 1.0
  _tkr = ticker(1); _tkr.capture_list(_imu.raw); _tkr.callback(_tick_imu); _tkr.start(5)
  t0 = ticks_ms()
  while not _imu.is_calibrated:
    sleep_ms(10)
    if ticks_diff(ticks_ms(), t0) > 10000: break
  print("[otto] IMU OK yaw=%.2f" % _yaw() if _imu.is_calibrated else "[otto] 标定超时")
  _motors = MotionControl(); _arb = MotorArbiter(_motors)
  print("[otto] 就绪. fwd/side/spin/orbit(d,s) / set_map(a,b,c)")

# ── 轮序映射 ──
def set_map(a, b, c):
  MotionControlOtto._MAP = (a, b, c)
  print("[otto] MAP=(%d,%d,%d)" % (a, b, c))

def set_scale(v):
  MotionControlOtto._SCALE = float(v)
  print("[otto] SCALE=%.2f" % v)

# ── 当前使用的运动学 ──
_use_otto = False

def _M():
  return MotionControlOtto if _use_otto else MotionControl

def use_otto(v=True):
  global _use_otto
  _use_otto = bool(v)
  print("[otto] use_otto=%s" % _use_otto)

# ── 打印 PWM ──
def _show(name, duties):
  global _pwm_cache
  _pwm_cache = list(duties)
  m = _motors
  pwm = []
  for i, d in enumerate(duties):
    pct = m._pct_to_pwm(float(d))
    pwm.append(pct)
  print("[%s] duties=[%.1f, %.1f, %.1f] pwm=[%d, %d, %d]" %
        (name, duties[0], duties[1], duties[2], pwm[0], pwm[1], pwm[2]))

# ── 测试 ──
def fwd(duty=30, ms=2000):
  _arb.acquire(_OWNER)
  y0 = _yaw()
  d = _M().move_forward(duty)
  _show("fwd", d)
  _arb.write(_OWNER, d)
  sleep_ms(ms)
  dy = wrap_deg(_yaw() - y0)
  _arb.hold_brake(_OWNER)
  print("[fwd] yaw Δ=%+.1f°  duty=+%d%%  看车朝哪走" % (dy, duty))

def side(duty=30, ms=2000):
  _arb.acquire(_OWNER)
  y0 = _yaw()
  d = _M().move_side(duty)
  _show("side", d)
  _arb.write(_OWNER, d)
  sleep_ms(ms)
  dy = wrap_deg(_yaw() - y0)
  _arb.hold_brake(_OWNER)
  print("[side] yaw Δ=%+.1f°  duty=+%d%%  看车往哪走" % (dy, duty))

def spin(duty=30, ms=500):
  _arb.acquire(_OWNER)
  y0 = _yaw()
  d = [float(duty), float(duty), float(duty)]
  _show("spin", d)
  _arb.write(_OWNER, d)
  sleep_ms(ms)
  dy = wrap_deg(_yaw() - y0)
  _arb.hold_brake(_OWNER)
  print("[spin] yaw Δ=%+.1f°  CW(yaw↓)" % dy)

def orbit(speed=25, spin_duty=-15, angle=-90, ms=2000):
  """绕轴转。Otto: angle控制绕哪个轴(-90=后轴, 0=前轴)  原版: angle=90=绕前轴"""
  _arb.acquire(_OWNER)
  y0 = _yaw()
  if _use_otto and hasattr(_M(), 'move_with_spin'):
    d = _M().move_with_spin(speed, angle, spin_duty)
  else:
    s = float(speed)
    side_d = MotionControl.move(s, float(angle))
    sp = float(spin_duty)
    d = [side_d[0] + sp, side_d[1] + sp, side_d[2] + sp]
  _show("orbit", d)
  _arb.write(_OWNER, d, False)
  sleep_ms(ms)
  dy = wrap_deg(_yaw() - y0)
  _arb.hold_brake(_OWNER)
  print("[orbit] yaw Δ=%+.1f°  speed=%d spin=%d angle=%d" % (dy, speed, spin_duty, angle))

def smooth_test(duty=30):
  """对比: 原版 vs Otto 四个方向"""
  print("=" * 50)
  print("  原版 MotionControl (已热补丁修正)")
  print("=" * 50)
  use_otto(False)
  fwd(duty, 500)
  side(duty, 500)
  spin(duty, 300)
  orbit(25, -15, 500)
  print()
  print("=" * 50)
  print("  Otto MotionControlOtto")
  print("=" * 50)
  use_otto(True)
  fwd(duty, 500)
  side(duty, 500)
  spin(duty, 300)
  orbit(25, -15, 500)
  print()
  use_otto(False)

init()
