"""
imu_test.py — IMU963 调参工具（极简版）

用法:
  >>> import imu_test
  >>> cal()      # 标定
  >>> mon(5)     # 5Hz 监控, Ctrl+C 停止
  >>> sc()       # 旋转360°刻度标定, Ctrl+C 停止
  >>> dr(30)     # 静止30s漂移测试
  >>> mag_cal()  # 硬铁标定: 旋转360°收集 min/max
  >>> mag()      # 磁力计监视: raw XYZ / mad vs mag
  >>> raw()      # 原始数据, Ctrl+C 停止
  >>> ref()      # 对准参考线,记录 yaw
  >>> check()    # 再次对准参考线,输出偏差
  >>> shake(8)   # 暴力来回转 8s, 测对称性
  >>> whirl(3600)# 快速旋转 3600°(10圈), 测漂移
  >>> spin2(20,15)# spin_beta 切换对称测试
  >>> goto(90)    # PID 闭环转到 yaw=90°, 测精度
  >>> set('gyro_scale', 1.135)  # 改参数
  >>> show()     # 查看参数
"""

from seekfree import IMU963RX
from smartcar import ticker
from imu import ImuSensor, MadgwickAHRS
from imu import _acc_to_g, _gyro_to_radps, GYRO_LSB_963, ACC_LSB_PER_G
import math, time
from time import ticks_ms, ticks_diff, sleep_ms
import gc

try:
  from motion import MotionControl, MotorArbiter, HeadingPID, wrap_deg
  _HAVE_MOTION = True
except Exception:
  _HAVE_MOTION = False

# ============================================================
# 全局状态
# ============================================================

_imu = None
_tkr = None
_motors = None
_arb = None
_OWNER = "TURN"

# 可调参数
gyro_scale = 1.0
beta        = 0.05
spin_beta   = 0.01
spin_dps    = 40.0
calib_n     = 200
gyro_still  = 0.0175
acc_still   = 0.05
bias_alpha  = 0.002
still_need  = 100

# 磁力计
mag_on      = False
mag_ox = mag_oy = mag_oz = 0.0
mag_alpha   = 0.002
mag_dead    = 2.2
mag_pull    = 6.7
mag_still_n = 100
mag_lpf     = 0.01

_tick_n = 0


def _tick(_):
  global _tick_n
  try:
    _imu.update()
    _tick_n += 1
  except Exception:
    pass


# ============================================================
# 初始化 + 标定
# ============================================================

def cal(n=200):
  """标定 IMU 陀螺 bias。n=采样帧数(200Hz)。"""
  global _imu, _tkr, calib_n
  calib_n = n

  print("[cal] init IMU963...")
  _imu = ImuSensor(calibrate_samples=n, beta=beta, model="963")
  _imu._gyro_scale = gyro_scale
  _imu._spin_beta = spin_beta
  _imu._spin_dps = spin_dps
  _imu._gyro_still = gyro_still
  _imu._acc_still = acc_still
  _imu._bias_alpha = bias_alpha
  _imu._still_needed = still_need
  _mag_sync()

  _tkr = ticker(1)
  _tkr.capture_list(_imu.raw)
  _tkr.callback(_tick)
  _tkr.start(5)  # 200Hz

  print("[cal] 标定中 (%d 采样)..." % n)
  t0 = ticks_ms()
  while not _imu.is_calibrated:
    sleep_ms(10)
    if ticks_diff(ticks_ms(), t0) > 10000:
      print("[cal] 超时!")
      break
  if _imu.is_calibrated:
    b = _imu._bias
    print("[cal] OK bias=[%.4f, %.4f, %.4f] rad/s" % (b[0], b[1], b[2]))
    print("[cal] yaw=%.2f" % _imu.get_yaw())


def _mag_sync():
  if _imu is None: return
  _imu._mag_enabled = mag_on
  _imu.set_mag_offset(mag_ox, mag_oy, mag_oz)
  _imu._mag_alpha = mag_alpha
  _imu._mag_dead = mag_dead
  _imu._mag_pull_max = mag_pull
  _imu._mag_still_need = mag_still_n
  _imu._mag_lpf_alpha = mag_lpf
  if mag_on:
    _imu.mag_enabled = True

def _yaw():   return _imu.get_yaw()
def _mad():   return _imu._filter.yaw_deg()
def _dps():   return _imu._gyro_dps
def _still(): return _imu._still_count
def _bias():  return _imu._bias
def _beta():  return _imu._filter.beta
def _off():   return _imu._fused_offset
def _mag():   return _imu.mag_data


# ============================================================
# 监控
# ============================================================

def mon(hz=2):
  """连续监控 yaw/dps/bias/beta。Ctrl+C 停止。"""
  dt = max(20, 1000 // hz)
  print("[mon] %d Hz  Ctrl+C 停止" % hz)
  try:
    while True:
      t = ticks_ms() / 1000.0
      s = "●" if _dps() >= spin_dps else ("○" if _dps() < 5 else "◐")
      print("[%7.2f] y=%+7.1f m=%+7.1f dps=%5.1f st=%4d β=%.3f %s" % (
        t, _yaw(), _mad(), _dps(), _still(), _beta(), s))
      sleep_ms(dt)
  except KeyboardInterrupt:
    print("[mon] stop")


def raw(hz=5):
  """原始陀螺/加计数据。Ctrl+C 停止。"""
  dt = max(20, 1000 // hz)
  print("[raw] %d Hz  Ctrl+C 停止" % hz)
  try:
    while True:
      d = _imu.data
      ax = d[0] / ACC_LSB_PER_G
      ay = d[1] / ACC_LSB_PER_G
      az = d[2] / ACC_LSB_PER_G
      am = math.sqrt(ax*ax + ay*ay + az*az)
      gx = d[3] / GYRO_LSB_963
      gy = d[4] / GYRO_LSB_963
      gz = d[5] / GYRO_LSB_963
      gm = math.sqrt(gx*gx + gy*gy + gz*gz)
      mx = d[6] if len(d) >= 9 else 0
      my = d[7] if len(d) >= 9 else 0
      mz = d[8] if len(d) >= 9 else 0
      print("[%7.2f] g=(%+6.0f,%+6.0f,%+6.0f) a=(%+5.2f,%+5.2f,%+5.2f) |g|=%.1f |a|=%.2f m=(%d,%d,%d) y=%+.1f" % (
        ticks_ms()/1000.0, d[3], d[4], d[5], ax, ay, az, gm*57.3, am, mx, my, mz, _yaw()))
      sleep_ms(dt)
  except KeyboardInterrupt:
    print("[raw] stop")


# ============================================================
# 刻度标定
# ============================================================

def sc(N=1):
  """陀螺刻度标定: 旋转 N×360°, Ctrl+C 停止看结果。"""
  start = _mad()
  accum = 0.0
  last = start
  target = 360.0 * N
  print("[sc] 旋转 %d×360° (目标 %.0f°)  Ctrl+C 结束" % (N, target))
  print("[sc] 起始 mad=%.2f" % start)
  _t_last = 0
  try:
    while True:
      y = _mad()
      accum += _imu._normalize_angle(y - last)
      last = y
      pct = abs(accum) / target * 100
      if ticks_diff(ticks_ms(), _t_last) > 1000:
        _t_last = ticks_ms()
        print("  acc=%.1f° / %.0f° (%.0f%%)  y=%.1f  dps=%.1f" % (accum, target, pct, y, _dps()))
      sleep_ms(50)
  except KeyboardInterrupt:
    pass

  ratio = target / max(abs(accum), 0.1)
  new_scale = gyro_scale * ratio
  print("[sc] ─────────────────")
  print("[sc] 目标:  %.1f°" % target)
  print("[sc] 测得:  %.1f°" % abs(accum))
  print("[sc] 修正比: %.4f  (%s)" % (ratio, "IMU偏慢→scale↑" if ratio > 1 else "IMU偏快→scale↓"))
  print("[sc] 旧 scale: %.4f" % gyro_scale)
  print("[sc] 新 scale: %.4f  ← set('gyro_scale', %.4f)" % (new_scale, new_scale))


# ============================================================
# 漂移测试
# ============================================================

def dr(sec=30):
  """静止漂移测试 sec 秒。"""
  print("[dr] 静止 %ds 测量漂移..." % sec)
  start = _yaw()
  samples = []
  t0 = ticks_ms()
  while True:
    elapsed = ticks_diff(ticks_ms(), t0) / 1000.0
    if elapsed >= sec:
      break
    if len(samples) < int(elapsed) + 1:
      y = _yaw()
      drift = _imu._normalize_angle(y - start)
      samples.append((elapsed, y, _mad(), _dps()))
      print("  t=%4.1f  y=%+7.2f  mad=%+7.2f  dps=%5.1f  drift=%+6.2f  st=%d" % (
        elapsed, y, samples[-1][2], samples[-1][3], drift, _still()))
    sleep_ms(100)

  n = len(samples)
  total = _imu._normalize_angle(samples[-1][1] - start)
  rate = total / samples[-1][0] if samples[-1][0] > 0 else 0
  dps_avg = sum(s[3] for s in samples) / n
  yaws = [s[1] for s in samples]
  pp = max(yaws) - min(yaws)
  print("[dr] ─────────────────")
  print("[dr] 时长 %ds 样本 %d" % (sec, n))
  print("[dr] 起始: %+.2f  结束: %+.2f" % (start, samples[-1][1]))
  print("[dr] 总漂移: %+.2f°  速率: %+.3f °/s (%+.1f °/min)" % (total, rate, rate*60))
  print("[dr] 峰峰: %.2f°  平均dps: %.2f" % (pp, dps_avg))
  if abs(rate) > 0.5:   print("[dr] ⚠ 漂移大 → 重标定")
  elif abs(rate) > 0.1:  print("[dr] ⚡ 漂移中等 → 加大 still_need / 减小 bias_alpha")
  else:                  print("[dr] ✓ 漂移正常")


# ============================================================
# 磁力计
# ============================================================

def mag(hz=2):
  """磁力计监视: raw XYZ / mad vs mag / offset。Ctrl+C 停止。"""
  if not _imu._mag_enabled:
    print("[mag] 磁力计未开 → 自动开启")
    global mag_on
    mag_on = True
    _mag_sync()
  dt = max(20, 1000 // hz)
  print("[mag] %d Hz  mad vs mag vs fused  |  raw(X,Y,Z)  Ctrl+C 停止" % hz)
  try:
    while True:
      mad_ = _mad()
      mh = _imu.get_mag_heading()
      fused = _imu._normalize_angle(mad_ + _off()) if mh is not None else mad_
      mrel = _imu.get_mag_rel()
      raw = _mag()
      print("[%7.2f] mad=%+7.1f  mag=%s  fused=%+7.1f  off=%+6.2f  |  raw=(%+5.0f,%+5.0f,%+5.0f)  st=%d" % (
        ticks_ms()/1000.0, mad_,
        "%+7.1f" % mh if mh is not None else "    n/a",
        fused, _off(),
        raw[0], raw[1], raw[2],
        _still()))
      sleep_ms(dt)
  except KeyboardInterrupt:
    print("[mag] stop")


def mag_cal(hz=10):
  """硬铁标定: 旋转机器人360°, 收集 min/max 算偏移。Ctrl+C 停止看结果。"""
  if not _imu._mag_enabled:
    global mag_on
    mag_on = True
    _mag_sync()
  dt = max(10, 1000 // hz)
  x_min = y_min = 9999.0
  x_max = y_max = -9999.0
  n = 0
  print("[mag_cal] %d Hz  旋转机器人 360° (慢速)  Ctrl+C 结束" % hz)
  print("[mag_cal] 收集 X Y 范围...")
  t_last = 0
  try:
    while True:
      raw = _mag()
      mx, my = raw[0], raw[1]
      if mx < x_min: x_min = mx
      if mx > x_max: x_max = mx
      if my < y_min: y_min = my
      if my > y_max: y_max = my
      n += 1
      now = ticks_ms()
      if ticks_diff(now, t_last) >= 500:
        t_last = now
        ox = (x_max + x_min) / 2.0
        oy = (y_max + y_min) / 2.0
        print("  X=[%.0f, %.0f]  Y=[%.0f, %.0f]  ox=%.1f  oy=%.1f  n=%d" % (
          x_min, x_max, y_min, y_max, ox, oy, n))
      sleep_ms(dt)
  except KeyboardInterrupt:
    pass
  ox = (x_max + x_min) / 2.0
  oy = (y_max + y_min) / 2.0
  print("[mag_cal] ─────────────────")
  print("[mag_cal] 样本 %d  X=[%.0f, %.0f]  Y=[%.0f, %.0f]" % (n, x_min, x_max, y_min, y_max))
  print("[mag_cal] 建议硬铁偏移:")
  print("[mag_cal]   set('mag_ox', %.1f)" % ox)
  print("[mag_cal]   set('mag_oy', %.1f)" % oy)
  if abs(ox) > 50 or abs(oy) > 50:
    print("[mag_cal] ⚠ 偏移 >50 → 检查附近有无磁铁/电机干扰")


# ============================================================
# 转动测试
# ============================================================

def spin(hz=10):
  """转动测试: 观察 spin_beta 切换。Ctrl+C 停止。"""
  dt = max(20, 1000 // hz)
  print("[spin] %d Hz  dps>%.0f→β=%.3f  Ctrl+C 停止" % (hz, spin_dps, spin_beta))
  try:
    while True:
      dps = _dps()
      y = _yaw()
      b = _beta()
      spinning = dps >= spin_dps
      tag = "●SPIN" if spinning else "○"
      print("[%7.2f] dps=%6.1f β=%.3f y=%+7.1f %s" % (ticks_ms()/1000.0, dps, b, y, tag))
      sleep_ms(dt)
  except KeyboardInterrupt:
    print("[spin] stop")


def spin2(hz=20, sec=10):
  """spin 切换对称度 + 末段静止稳定性。
     · enter==exit 算切换对称(滞回内)
     · 末尾 0.5s 真静下来时,yaw 峰峰 ≈ 静止稳态精度"""
  hz = max(2, hz); sec = max(2, sec)
  dt = max(10, 1000 // hz)
  end = ticks_ms() + sec * 1000
  enter_n = exit_n = 0
  in_spin = False
  pre_yaw = _yaw()
  print("[spin2] %d Hz  时长 %ds  spin_dps=%.0f  Ctrl+C 提前停" % (hz, sec, spin_dps))
  try:
    while ticks_diff(end, ticks_ms()) > 0:
      now_in = _dps() >= spin_dps
      if now_in and not in_spin: enter_n += 1
      elif not now_in and in_spin: exit_n += 1
      in_spin = now_in
      sleep_ms(dt)
  except KeyboardInterrupt:
    pass
  # 末尾 0.5s 真静下来那段:用于测静止稳态精度
  seg = []
  t0 = ticks_ms()
  while ticks_diff(ticks_ms(), t0) < 500:
    seg.append(_yaw())
    sleep_ms(20)
  post_yaw = _yaw()
  total_rot = _imu._normalize_angle(post_yaw - pre_yaw)
  print("[spin2] ─────────────────")
  print("[spin2] enter=%d  exit=%d  (差=%d, 0=对称)" % (
    enter_n, exit_n, enter_n - exit_n))
  print("[spin2] 本次净转角=%+.1f° (起→终 yaw 差,跟 spin_beta 无关)" % total_rot)
  pp = None
  if seg:
    pp = abs(_imu._normalize_angle(seg[-1] - seg[0]))
    print("[spin2] 末段 0.5s 静止 yaw 峰峰=%.2f°" % pp)
  if enter_n == 0:
    print("[spin2] ⚠ 未触发 spin_beta → set('spin_dps', 20) 拉低门槛再试")
  elif enter_n != exit_n:
    print("[spin2] ⚠ enter≠exit → 卡在 spin,检查停手是否真到 %.0f dps 以下" % (spin_dps*0.5))
  else:
    if pp is None:
      print("[spin2] ✓ 切换对称 (末段采样不足,未测稳定性)")
    elif pp <= 0.5:
      print("[spin2] ✓ 切换对称 + 末段稳 (<=0.5°)")
    elif pp <= 1.5:
      print("[spin2] ⚡ 切换对称,末段轻抖 (<=1.5°, 可接受)")
    else:
      print("[spin2] ⚠ 末段漂 >1.5° → bias_alpha 可能偏小 或 spin_beta 偏大")


def _init_motion():
  """懒初始化 MotionControl + MotorArbiter,只在 turn() 第一次被调时建。"""
  global _motors, _arb
  if _motors is not None:
    return True
  if not _HAVE_MOTION:
    print("[motion] motion 模块不可用,检查 motion.py")
    return False
  try:
    _motors = MotionControl()
    _arb = MotorArbiter(_motors)
    print("[motion] 初始化 OK")
    return True
  except Exception as e:
    print("[motion] init err: %s" % e)
    return False


def turn(deg=360.0, max_duty=40.0, kp=1.6, sign=1.0):
  """yaw 闭环转 deg 度(<0 反向)。用累积角 cum(自动跨 ±180),PID=P。
     sign= 旋转极性(+1/-1)用于匹配电机的'正方向',默认 +1。
     完成或 Ctrl+C 自动 force_brake,转完恢复原 mag 状态。"""
  if not _init_motion():
    return
  if _imu is None or not _imu.is_calibrated:
    print("[turn] IMU 未标定,先 cal()")
    return
  deg = float(deg)
  if abs(deg) < 5:
    print("[turn] deg 太小(<5°),无意义")
    return
  sgn = float(sign) * (1.0 if deg > 0 else -1.0)  # 期望累积方向
  target_cum = abs(deg)
  saved_mag = mag_on
  if mag_on:
    set('mag_on', False)  # 转中关磁,防止静止瞬间磁拉拽
  _arb.acquire(_OWNER)
  pid = HeadingPID(kp=float(kp), max_output=float(max_duty), deadband=1.0)
  cum = 0.0
  last_yaw = _imu.get_yaw()
  t0 = ticks_ms()
  last_print = t0
  arb_writes = 0
  arb_rejects = 0
  print("[turn] target=%+.1f°  sign=%+.0f  kp=%.2f  duty≤%.0f%%  Ctrl+C 停" % (
    deg, sgn, kp, max_duty))
  try:
    while True:
      cur = _imu.get_yaw()
      d = _imu._normalize_angle(cur - last_yaw)
      cum += d
      last_yaw = cur
      remain = target_cum - abs(cum)
      now_ms = ticks_ms()
      dt = ticks_diff(now_ms, t0) / 1000.0
      if dt <= 0 or dt > 0.5:
        dt = 0.05
      if remain <= 1.5:
        break
      err = sgn * remain
      rot = pid.update(err, dt)
      ret = _arb.write(_OWNER, [rot, rot, rot])
      if ret: arb_writes += 1
      else: arb_rejects += 1
      if ticks_diff(now_ms, last_print) >= 200:
        last_print = now_ms
        print("[%5.2fs] yaw=%+7.2f  cum=%+7.1f°  remain=%+5.1f°  rot=%+5.1f%%  dps=%5.1f  w=%d r=%d" % (
          dt, cur, cum, remain, rot, _imu._gyro_dps, arb_writes, arb_rejects))
      sleep_ms(20)
  except KeyboardInterrupt:
    pass
  finally:
    try:
      _arb.force_brake()
    except Exception:
      pass
    if saved_mag and mag_on is False:
      set('mag_on', True)
  print("[turn] ─────────────────")
  print("[turn] 目标累计 %+.1f°  实转 %+.1f°  残量 %+.1f° (cum 是 sign 方向)" % (
    sgn * target_cum, cum, target_cum - abs(cum)))
  print("[turn] arb: writes=%d rejects=%d" % (arb_writes, arb_rejects))


# ============================================================
# 参考线
# ============================================================

_ref_yaw = None
_ref_mad = None
_ref_ms = 0

def ref():
  """对准参考线后调用，记录当前 yaw，同时重置磁参考。"""
  global _ref_yaw, _ref_mad, _ref_ms
  if _imu is None:
    print("[ref] 请先 cal()")
    return
  _ref_yaw = _yaw()
  _ref_mad = _mad()
  _ref_ms = ticks_ms()
  # 重置磁参考 → 当前位置即磁"零位"
  if _imu._mag_enabled:
    _imu._mag_ref = None
    _imu._mag_rel_lpf = None
    _imu._fused_offset = 0.0
    print("[ref] yaw=%+.2f  mad=%+.2f  ✓ 已记录 (磁参考已重置)" % (_ref_yaw, _ref_mad))
  else:
    print("[ref] yaw=%+.2f  mad=%+.2f  ✓ 已记录" % (_ref_yaw, _ref_mad))

def check():
  """再次对准参考线后调用，输出与 ref() 的偏差。"""
  if _ref_yaw is None:
    print("[check] 请先 ref()")
    return
  y, m = _yaw(), _mad()
  dy = _imu._normalize_angle(y - _ref_yaw)
  dm = _imu._normalize_angle(m - _ref_mad)
  dt = ticks_diff(ticks_ms(), _ref_ms) / 1000.0
  print("[check] 偏差 yaw=%+.2f°  mad=%+.2f°  距ref %.1fs" % (dy, dm, dt))
  if abs(dy) <= 1.0:   print("[check] ✓ 优秀 (<1°)")
  elif abs(dy) <= 3.0:  print("[check] ⚡ 可接受 (<3°)")
  else:                 print("[check] ⚠ 偏差大 → 检查参数")


# ============================================================
# 暴力来回转
# ============================================================

def shake(sec=8, duty=60, period_ms=150):
  """暴力来回转: 三轮回转交替正反转, 测对称性。Ctrl+C 停止。"""
  if not _init_motion(): return
  if _imu is None or not _imu.is_calibrated:
    print("[shake] IMU 未标定,先 cal()")
    return
  sid = "SHAKE"
  _arb.acquire(sid)
  sy, sm = _yaw(), _mad()
  last = sm
  cum_abs = 0.0
  t0 = ticks_ms()
  end_ms = t0 + int(sec * 1000)
  cycles = 0
  d = int(max(10, min(100, duty)))
  p = max(30, int(period_ms))
  _chi = _imu._normalize_angle
  print("[shake] 来回转 %ds  duty=%d  period=%dms  Ctrl+C 停止" % (sec, d, p))
  try:
    while ticks_diff(end_ms, ticks_ms()) > 0:
      _arb.write(sid, [d, d, d])
      td = ticks_ms() + p
      while ticks_diff(td, ticks_ms()) > 0:
        cur = _mad(); delta = _chi(cur - last)
        cum_abs += abs(delta); last = cur
        sleep_ms(10)
      _arb.write(sid, [-d, -d, -d])
      td = ticks_ms() + p
      while ticks_diff(td, ticks_ms()) > 0:
        cur = _mad(); delta = _chi(cur - last)
        cum_abs += abs(delta); last = cur
        sleep_ms(10)
      cycles += 1
  except KeyboardInterrupt:
    pass
  finally:
    _arb.force_brake()
  ey, em = _yaw(), _mad()
  net_y = _chi(ey - sy)
  net_m = _chi(em - sm)
  et = max(ticks_diff(ticks_ms(), t0) / 1000.0, 0.01)
  print("[shake] ─────────────────")
  print("[shake] %.1fs  cycles=%d  duty=%d" % (et, cycles, d))
  print("[shake] 起始 y=%+.2f m=%+.2f → 结束 y=%+.2f m=%+.2f" % (sy, sm, ey, em))
  print("[shake] 净转角 yaw=%+.2f°  mad=%+.2f°" % (net_y, net_m))
  print("[shake] 累积|转角|≈%.0f°  平均 dps=%.0f" % (cum_abs, cum_abs / et))
  if abs(net_y) <= 1.0:   print("[shake] ✓ 来回对称优秀")
  elif abs(net_y) <= 3.0:  print("[shake] ⚡ 轻微不对称")
  else:                    print("[shake] ⚠ 不对称 >3° → 检查 gyro_scale / spin_beta")


# ============================================================
# 快速旋转 3600°
# ============================================================

def whirl(deg=3600, duty=70):
  """向一个方向旋转 deg 度(默认3600=10圈), 看 yaw 偏差。Ctrl+C 停止。"""
  if not _init_motion(): return
  if _imu is None or not _imu.is_calibrated:
    print("[whirl] IMU 未标定,先 cal()")
    return
  deg = float(deg)
  if abs(deg) < 180:
    print("[whirl] deg 太小(<180°)")
    return
  d = int(max(10, min(100, duty)))
  sgn = 1 if deg > 0 else -1
  target = abs(deg)
  sid = "WHIRL"
  _arb.acquire(sid)
  sy, sm = _yaw(), _mad()
  cum = 0.0
  last = sm
  t0 = ticks_ms()
  last_print = t0
  _chi = _imu._normalize_angle
  print("[whirl] 目标 %+.0f° (%.1f圈)  duty=%d  Ctrl+C 停止" % (deg, target / 360.0, d))
  try:
    while cum < target:
      cur = _mad(); cum += abs(_chi(cur - last)); last = cur
      _arb.write(sid, [d * sgn, d * sgn, d * sgn])
      now = ticks_ms()
      if ticks_diff(now, last_print) >= 500:
        last_print = now
        et = max(ticks_diff(now, t0) / 1000.0, 0.01)
        print("  累计 %.0f° / %.0f° (%.0f%%)  dps=%.0f  yaw=%+.1f" % (
          cum, target, cum / target * 100, cum / et, _yaw()))
      sleep_ms(20)
  except KeyboardInterrupt:
    pass
  finally:
    _arb.force_brake()
  ey, em = _yaw(), _mad()
  net_y = _chi(ey - sy)
  net_m = _chi(em - sm)
  et = max(ticks_diff(ticks_ms(), t0) / 1000.0, 0.01)
  print("[whirl] ─────────────────")
  print("[whirl] 目标 %+.0f°  累计 %.0f°  耗时 %.1fs" % (deg, cum, et))
  print("[whirl] 起始 y=%+.2f m=%+.2f → 结束 y=%+.2f m=%+.2f" % (sy, sm, ey, em))
  print("[whirl] yaw 偏差 %+.2f°  mad 偏差 %+.2f°  (期望≈0°)" % (net_y, net_m))
  if abs(net_y) <= 2.0:   print("[whirl] ✓ 优秀 (<2°)")
  elif abs(net_y) <= 5.0:  print("[whirl] ⚡ 可接受 (<5°)")
  elif abs(net_y) <= 15.0: print("[whirl] ⚠ 偏大 → 调 gyro_scale")
  else:                    print("[whirl] ❌ 严重漂移 → 重标定 + 调 gyro_scale")


# ============================================================
# PID 闭环定点旋转 — 测 IMU+电机配合精度
# ============================================================

def goto(target, max_duty=40, kp=1.0, kd=0.08, tol=1.5):
  """PD 闭环转到绝对 yaw=target 度。D 项抑振荡, 最小占空比破静摩擦。Ctrl+C 停止。"""
  if not _init_motion(): return
  if _imu is None or not _imu.is_calibrated:
    print("[goto] 请先 cal()")
    return
  target = float(target)
  sid = "GOTO"
  _arb.acquire(sid)
  sy = _yaw()
  _chi = _imu._normalize_angle
  t0 = ticks_ms()
  last_print = t0
  settle = 0
  NEED = 10
  d_max = max(10, min(100, int(max_duty)))
  prev_yaw = sy
  dps_est = 0.0

  print("[goto] → %+.1f°  max=%d  kp=%.2f  kd=%.3f  tol=%.1f°  Ctrl+C 停止" % (
    target, d_max, kp, kd, tol))
  try:
    while True:
      cur = _yaw()
      err = _chi(cur - target)        # + = 已过目标(CW侧)
      dps_est = _chi(cur - prev_yaw) / 0.02

      if abs(err) <= tol:
        settle += 1
        if settle >= NEED:
          break
        # 容忍区内仅 D 制动，消残余动量
        out = -kd * dps_est
        if abs(out) < 3:
          out = 0
      else:
        settle = 0
        out = kp * err + kd * dps_est
        if out > d_max: out = d_max
        elif out < -d_max: out = -d_max
        # 最小占空比克服静摩擦 (3轮 ~6-7%, 留1%余量)
        if 0 < out < 7:
          out = 7
        elif 0 > out > -7:
          out = -7

      _arb.write(sid, [out, out, out])
      prev_yaw = cur
      now = ticks_ms()
      if ticks_diff(now, last_print) >= 200:
        last_print = now
        print("  t=%.1fs  yaw=%+.1f  err=%+.1f°  dps=%.0f  out=%.0f  st=%d" % (
          ticks_diff(now, t0) / 1000.0, cur, err, _dps(), out, settle))
      sleep_ms(20)
  except KeyboardInterrupt:
    pass
  finally:
    _arb.force_brake()

  ey = _yaw()
  ef = _chi(ey - target)
  et = max(ticks_diff(ticks_ms(), t0) / 1000.0, 0.01)
  print("[goto] ─────────────────")
  print("[goto] %.1fs  起始=%+.1f → 目标=%+.1f → 最终=%+.1f  误差=%+.2f°" % (
    et, sy, target, ey, ef))
  if _ref_yaw is not None:
    dr = _chi(ey - _ref_yaw)
    print("[goto] 参考线 yaw=%+.1f  当前 vs 参考线偏差=%+.2f°" % (_ref_yaw, dr))
  if abs(ef) <= 1.5:   print("[goto] ✓ 优秀")
  elif abs(ef) <= 4.0:  print("[goto] ⚡ 可接受")
  else:                 print("[goto] ⚠ 偏差大 → 调 kp / kd / max_duty")


# ============================================================
# 参数持久化
# ============================================================

_PARAM_FILE = "/flash/imu_test_params.txt"

def save():
  """保存参数到 /flash/imu_test_params.txt"""
  ks = ['gyro_scale','beta','spin_beta','spin_dps',
        'gyro_still','acc_still','bias_alpha','still_need',
        'mag_on','mag_ox','mag_oy','mag_oz',
        'mag_alpha','mag_dead','mag_pull','mag_still_n','mag_lpf']
  try:
    with open(_PARAM_FILE, "w") as f:
      for k in ks:
        v = globals()[k]
        f.write("%s=%s\n" % (k, v))
    print("[save] → %s" % _PARAM_FILE)
  except OSError as e:
    print("[save] 失败: %s" % e)

def show():
  """打印全部参数。"""
  ks = ['gyro_scale','beta','spin_beta','spin_dps','calib_n',
        'gyro_still','acc_still','bias_alpha','still_need',
        'mag_on','mag_ox','mag_oy','mag_oz',
        'mag_alpha','mag_dead','mag_pull','mag_still_n','mag_lpf']
  print("[show] ─── 参数 ───")
  for k in ks:
    print("  %-16s = %s" % (k, globals()[k]))
  if _imu:
    b = _bias()
    print("  bias            = [%.4f, %.4f, %.4f]" % (b[0], b[1], b[2]))
    print("  yaw             = %.2f" % _yaw())

def load():
  """从 /flash/imu_test_params.txt 加载参数"""
  try:
    with open(_PARAM_FILE, "r") as f:
      for line in f:
        line = line.strip()
        if not line or '=' not in line: continue
        k, v = line.split('=', 1)
        if k not in globals(): continue
        old = globals()[k]
        t = type(old)
        try:
          if t is bool: v2 = str(v).lower() in ('1','true','yes','on')
          elif t is int: v2 = int(float(v))
          else: v2 = t(float(v))
          globals()[k] = v2
        except: pass
    print("[load] ← %s" % _PARAM_FILE)
    show()
  except OSError:
    print("[load] 无存档")

# 启动时自动加载
load()

def set(k, v):
  """设参数: set('gyro_scale', 1.135)"""
  g = globals()
  if k not in g:
    print("[set] 未知: %s" % k)
    return
  old = g[k]
  t = type(old)
  try:
    if t is bool: v2 = str(v).lower() in ('1','true','yes','on')
    elif t is int: v2 = int(float(v))
    else: v2 = t(float(v))
  except:
    print("[set] 类型错误")
    return
  g[k] = v2
  # 同步到 IMU
  if _imu:
    _sync = {
      'gyro_scale': lambda x: setattr(_imu, '_gyro_scale', x),
      'beta': lambda x: setattr(_imu._filter, 'beta', x),
      'spin_beta': lambda x: setattr(_imu, '_spin_beta', x),
      'spin_dps': lambda x: setattr(_imu, '_spin_dps', x),
      'gyro_still': lambda x: setattr(_imu, '_gyro_still', x),
      'acc_still': lambda x: setattr(_imu, '_acc_still', x),
      'bias_alpha': lambda x: setattr(_imu, '_bias_alpha', x),
      'still_need': lambda x: setattr(_imu, '_still_needed', x),
    }
    if k in _sync: _sync[k](v2)
    if k.startswith('mag_'): _mag_sync()
  print("[set] %s: %s → %s" % (k, old, v2))
  _auto_save()

def _auto_save():
  """set() 后自动持久化到 /flash/imu_test_params.txt"""
  ks = ['gyro_scale','beta','spin_beta','spin_dps',
        'gyro_still','acc_still','bias_alpha','still_need',
        'mag_on','mag_ox','mag_oy','mag_oz',
        'mag_alpha','mag_dead','mag_pull','mag_still_n','mag_lpf']
  try:
    with open(_PARAM_FILE, "w") as f:
      for k in ks:
        f.write("%s=%s\n" % (k, globals()[k]))
  except Exception:
    pass


print("[imu_test] 自动标定中...")
cal()
print("[imu_test] 就绪. ref→goto(90)/shake/whirl→check / mon(5) / sc(1) / dr(30)")
