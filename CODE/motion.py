"""
motion.py — 电机驱动 + 仲裁 + 航向PID + 目标筛选
"""

from machine import PWM
from time import sleep_us, ticks_ms, ticks_diff
import math
from log import info


# =============================================================================
#                          MotionControl — 三轮 PWM
# =============================================================================

class MotionControl:
  """
  DRV8870DDAR 驱动，每路电机用两个 PWM 组成半桥:
    - PWM_A (CCW/Pin1): 逆时针PWM — 占空比控制速度，HIGH(65535)=OFF
    - PWM_B (CW/Pin2):  顺时针PWM — 占空比控制速度，HIGH(65535)=OFF

  方向逻辑 (针对 DRV8870 的 IN1/IN2):
    IN1=PWM, IN2=HIGH → 顺时针 (电流 IN1→IN2), 实际占空比 = 100-PWM%
    IN1=HIGH, IN2=PWM → 逆时针 (电流 IN2→IN1), 实际占空比 = 100-PWM%
    IN1=LOW,  IN2=LOW  → 滑行停止 (Coast)
    IN1=HIGH, IN2=HIGH → 电子刹车 (Brake)

  默认 13kHz PWM，占空比范围 [-100, 100] 百分比。
  """

  def __init__(self):
    self._motors = [
      (PWM('D6', 13000, duty_u16=0), PWM('D7', 13000, duty_u16=0)),
      (PWM('D5', 13000, duty_u16=0), PWM('D4', 13000, duty_u16=0)),
      (PWM('C28', 13000, duty_u16=0), PWM('C29', 13000, duty_u16=0)),
    ]

  # ——————————————————————————————————————————————————————————
  #                         电机控制
  # ——————————————————————————————————————————————————————————
  def setSpeed(self, duties):
    """
    设置三路电机占空比 [-100, 100]。
    正值=逆时针, 负值=顺时针, 0=停。
    ! 传入数组不应大于电机数,这里不作校验
    """
    for i, d in enumerate(duties):
      d=max(-100,min(100,int(d)));ccw,cw=self._motors[i]
      if d>0:ccw.duty_u16(self._pct_to_pwm(d));sleep_us(76);cw.duty_u16(65535)
      elif d<0:ccw.duty_u16(65535);sleep_us(76);cw.duty_u16(self._pct_to_pwm(-d))
      else:ccw.duty_u16(0);sleep_us(76);cw.duty_u16(0)

  def brake(self):
    for ccw, cw in self._motors:
      ccw.duty_u16(65535)
      sleep_us(76)
      cw.duty_u16(65535)

  # ——————————————————————————————————————————————————————————
  #                         工具函数
  # ——————————————————————————————————————————————————————————
  @staticmethod
  def _pct_to_pwm(pct):
    return int((100 - max(0, min(100, pct))) * 65535 / 100)

  @staticmethod
  def move(speed, angle):
    """
    全向轮运动学逆解。
    speed: 合成速度大小
    angle: 移动方向角度 (deg), 0 = 机器人前方
    返回 [M1, M2, M3] 需要的占空比。
    """
    r = math.radians(-angle)
    c = math.cos(r) / math.sqrt(3)
    s = math.sin(r) / 3
    return [speed*(s+c),speed*(s-c),speed*(-2*s)]


# =============================================================================
#                          MotorArbiter — 电机唯一写入口
# =============================================================================

class MotorArbiter:
  def __init__(self, motors):
    self._motors = motors
    self._owner = None
    self._d0 = self._d1 = self._d2 = 0.0
    self._last_owner_warn_ms = 0

  @property
  def owner(self): return self._owner

  @property
  def duties(self):
    return self._d0, self._d1, self._d2

  @property
  def motors_active(self):
    return abs(self._d0) > 1.0 or abs(self._d1) > 1.0 or abs(self._d2) > 1.0

  def acquire(self, cid):
    if self._owner is not None and self._owner != cid:
      self._motors.brake()
      self._d0 = self._d1 = self._d2 = 0.0
    self._owner = cid

  def release(self, cid):
    if self._owner == cid:
      self._motors.brake()
      self._d0 = self._d1 = self._d2 = 0.0
      self._owner = None

  def write(self, cid, duties):
    if self._owner == cid:
      self._motors.setSpeed(duties)
      self._d0, self._d1, self._d2 = float(duties[0]), float(duties[1]), float(duties[2])
      return True
    now = ticks_ms()
    if ticks_diff(now, self._last_owner_warn_ms) >= 1000:
      self._last_owner_warn_ms = now
      info("ARB", "reject write owner=%s caller=%s" % (self._owner, cid))
    return False

  def hold_brake(self, cid):
    if self._owner == cid:
      self._motors.brake()
      self._d0 = self._d1 = self._d2 = 0.0

  def force_brake(self):
    self._motors.brake()
    self._d0 = self._d1 = self._d2 = 0.0
    self._owner = None


# =============================================================================
#                             HeadingPID — 航向 PID
# =============================================================================

class HeadingPID:
  def __init__(self, kp=2.0, ki=0.0, kd=0.0, max_output=100.0, deadband=0.0,
               gains=None):
    self._g = gains
    self.kp = kp; self.ki = ki; self.kd = kd
    self.max_output = max_output; self.deadband = deadband
    self._integral = 0.0; self._prev_error = 0.0; self._first_update = True

  def _params(self):
    g = self._g
    if g is not None:
      return g.kp, g.ki, g.kd, g.max_out, g.deadband
    return self.kp, self.ki, self.kd, self.max_output, self.deadband

  def update(self, error, dt):
    kp, ki, kd, mx, db = self._params()
    if abs(error) < db: error = 0.0
    if self._first_update:
      self._prev_error = error; self._first_update = False
      return 0.0
    self._integral += error * dt
    d = (error - self._prev_error) / dt if dt > 1e-6 else 0.0
    out = kp * error + ki * self._integral + kd * d
    # 反算钳位：输出饱和时立即修正积分，避免恢复缓慢
    if out > mx:
      out = mx
      if ki > 1e-9:
        self._integral = (mx - kp * error - kd * d) / ki
    elif out < -mx:
      out = -mx
      if ki > 1e-9:
        self._integral = (-mx - kp * error - kd * d) / ki
    self._prev_error = error
    return out

  def reset(self):
    self._integral = 0.0; self._prev_error = 0.0; self._first_update = True


# =============================================================================
#                           select_target — 目标筛选
# =============================================================================

def select_target(detections, cfg):
  if not detections: return None
  tc, mc = int(cfg.tracking.target_class), cfg.tracking.min_confidence
  allow = getattr(cfg, "match_allow", None)
  candidates = []
  for d in detections:
    cid, sc = d[0], d[1]
    if allow is not None:
      if cid not in allow: continue
    elif tc != 7 and cid != tc: continue
    if sc < mc: continue
    if d[4] <= 0 or d[5] <= 0: continue
    candidates.append(d)
  if not candidates: return None
  candidates.sort(key=lambda x: (0 if x[0] == tc or tc == 7 else 1, -x[8]))
  return candidates[0]
