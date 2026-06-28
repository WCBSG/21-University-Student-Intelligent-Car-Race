from machine import PWM
from time import sleep_us
import math

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
    self._speeds = [0, 0, 0]

  # ——————————————————————————————————————————————————————————
  #                         电机控制
  # ——————————————————————————————————————————————————————————
  def setSpeed(self, duties):
    """
    设置三路电机占空比 [-100, 100]。
    正值=逆时针, 负值=顺时针, 0=停。
    返回当前速度列表。
    """
    for i, d in enumerate(duties):
      d=self._clamp(int(d),-100,100);self._speeds[i]=d;ccw,cw=self._motors[i]
      if d>0:ccw.duty_u16(self._pct_to_pwm(d));sleep_us(76);cw.duty_u16(65535)
      elif d<0:ccw.duty_u16(65535);sleep_us(76);cw.duty_u16(self._pct_to_pwm(-d))
      else:ccw.duty_u16(0);sleep_us(76);cw.duty_u16(0)
    return self._speeds.copy()

  def addSpeed(self, duties):
    for i, d in enumerate(duties):
      self._speeds[i] = self._clamp(self._speeds[i] + int(d), -100, 100)
    return self.setSpeed(self._speeds)

  def stop(self):
    self._speeds = [0, 0, 0]
    for ccw, cw in self._motors:
      ccw.duty_u16(0)
      sleep_us(76)
      cw.duty_u16(0)

  def brake(self):
    self._speeds = [0, 0, 0]
    for ccw, cw in self._motors:
      ccw.duty_u16(65535)
      sleep_us(76)
      cw.duty_u16(65535)

  # ——————————————————————————————————————————————————————————
  #                         数据获取
  # ——————————————————————————————————————————————————————————
  def getSpeeds(self):
    return self._speeds.copy()

  def getRawPWM(self):
    return [(m[0].duty_u16(), m[1].duty_u16()) for m in self._motors]

  # ——————————————————————————————————————————————————————————
  #                         工具函数
  # ——————————————————————————————————————————————————————————
  @staticmethod
  def _pct_to_pwm(pct):
    return int((100 - max(0, min(100, pct))) * 65535 / 100)

  @staticmethod
  def _clamp(val, lo, hi):
    if val < lo:return lo
    if val > hi:return hi
    return val

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
