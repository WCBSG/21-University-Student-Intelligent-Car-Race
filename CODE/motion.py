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
  MIN_DUTY = 7  # 最低占空比(%) — 克服静摩擦，3轮需6-7%

  def setSpeed(self, duties, use_min_duty=True):
    """
    设置三路电机占空比 [-100, 100]。
    正值=逆时针, 负值=顺时针, 0=停。
    use_min_duty=False 可关闭 MIN_DUTY 提升，避免小修正失真。
    先写三路第一脚，统一 dead-time 后再写第二脚（DRV8870 需要 ≥76us）。
    """
    second = []
    for i, d in enumerate(duties):
      d = max(-100, min(100, int(d)))
      # 最低占空比：非零时至少 MIN_DUTY% 克服静摩擦
      if use_min_duty:
        if 0 < d < self.MIN_DUTY:
          d = self.MIN_DUTY
        elif -self.MIN_DUTY < d < 0:
          d = -self.MIN_DUTY
      ccw, cw = self._motors[i]
      if d > 0:
        ccw.duty_u16(self._pct_to_pwm(d))
        second.append((cw, 65535))
      elif d < 0:
        ccw.duty_u16(65535)
        second.append((cw, self._pct_to_pwm(-d)))
      else:
        ccw.duty_u16(0)
        second.append((cw, 0))
    sleep_us(76)
    for pin, val in second:
      pin.duty_u16(val)

  def brake(self):
    for ccw, cw in self._motors:
      ccw.duty_u16(65535)
    sleep_us(76)
    for ccw, cw in self._motors:
      cw.duty_u16(65535)

  # ——————————————————————————————————————————————————————————
  #                         工具函数
  # ——————————————————————————————————————————————————————————
  @staticmethod
  def _pct_to_pwm(pct):
    return int((100 - max(0, min(100, pct))) * 65535 / 100)

  _FWD_K = 1.0 / math.sqrt(3.0)   # ≈0.5774  — cos(0)/√3
  _SIDE_K = 1.0 / 3.0             # ≈0.3333  — sin(90)/3

  @staticmethod
  def move(speed, angle):
    """
    全向轮运动学逆解。
    speed: 合成速度大小
    angle: 移动方向角度 (deg), 0 = 机器人前方 , 逆时针增加度数
    返回 [M1, M2, M3] 需要的占空比。
    """
    r = math.radians(-angle)
    c = math.cos(r) / math.sqrt(3)
    s = math.sin(r) / 3
    return [speed*(s+c),speed*(s-c),speed*(-2*s)]

  @staticmethod
  def move_forward(speed):
    """前向运动学，免 trig。speed>0=前进。"""
    s = float(speed) * MotionControl._FWD_K
    return [s, -s, 0.0]

  @staticmethod
  def move_side(speed):
    """横向运动学，免 trig。speed>0=右移(=move(speed,-90))。"""
    s = float(speed) * MotionControl._SIDE_K
    return [s, s, -2.0 * s]


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

  def write(self, cid, duties, use_min_duty=True):
    if self._owner == cid:
      self._motors.setSpeed(duties, use_min_duty)
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
  """PD 航向控制器。rate 为 signed yaw 变化率(°/s)，D 项抑振荡。"""
  def __init__(self, kp=2.0, max_output=100.0, deadband=0.0, kd=0.0, gains=None):
    self._g = gains
    self.kp = kp
    self.max_output = max_output; self.deadband = deadband
    self.kd = kd

  def _params(self):
    g = self._g
    if g is not None:
      return (g.kp, g.max_out, g.deadband,
              getattr(g, "kd", 0.0))
    return self.kp, self.max_output, self.deadband, self.kd

  def update(self, error, dt, rate=0.0):
    """rate: signed yaw 变化率(°/s), 正=CCW。D 项=-kd*rate 抑过冲。"""
    kp, mx, db, kd = self._params()
    if abs(error) < db:
      return 0.0
    out = kp * error - kd * rate
    if out > mx:
      out = mx
    elif out < -mx:
      out = -mx
    return out

  def reset(self):
    pass


# =============================================================================
#                           角度 / 目标筛选
# =============================================================================

def wrap_deg(a):
  while a > 180.0:
    a -= 360.0
  while a < -180.0:
    a += 360.0
  return a


def select_target(detections, cfg, allow=None, target_class=None):
  """allow / target_class 由 MatchRunner 注入；未传则用 cfg 默认值。"""
  if not detections:
    return None
  tc = int(cfg.tracking_target_class) if target_class is None else int(target_class)
  mc = cfg.tracking_min_confidence
  candidates = []
  for d in detections:
    cid, sc = d[0], d[1]
    if allow is not None:
      if cid not in allow:
        continue
    elif tc != 7 and cid != tc:
      continue
    if sc < mc:
      continue
    if d[4] <= 0 or d[5] <= 0:
      continue
    candidates.append(d)
  if not candidates:
    return None
  candidates.sort(key=lambda x: (0 if x[0] == tc or tc == 7 else 1, -x[8]))
  return candidates[0]
