from machine import PWM
from time import sleep_us, ticks_ms, ticks_diff
import math
from log import info
class MotionControl:
  def __init__(self):
    self._motors = [
      (PWM('D6', 13000, duty_u16=0), PWM('D7', 13000, duty_u16=0)),
      (PWM('D5', 13000, duty_u16=0), PWM('D4', 13000, duty_u16=0)),
      (PWM('C28', 13000, duty_u16=0), PWM('C29', 13000, duty_u16=0)),
    ]
  MIN_DUTY = 7
  def setSpeed(self, duties, use_min_duty=True):
    second = []
    if len(duties) != 3:
      raise ValueError("three motor duties required")
    for i, d in enumerate(duties):
      d = max(-100, min(100, int(d)))
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
  @staticmethod
  def _pct_to_pwm(pct):
    return int((100 - max(0, min(100, pct))) * 65535 / 100)
  _FWD_K = 1.0 / math.sqrt(3.0)
  _SIDE_K = 1.0 / 3.0
  @staticmethod
  def move(speed, angle):
    r = math.radians(-angle)
    c = math.cos(r) / math.sqrt(3)
    s = math.sin(r) / 3
    return [speed*(s+c),speed*(s-c),speed*(-2*s)]
  @staticmethod
  def move_forward(speed):
    s = float(speed) * MotionControl._FWD_K
    return [s, -s, 0.0]
  @staticmethod
  def move_side(speed):
    s = float(speed) * MotionControl._SIDE_K
    return [s, s, -2.0 * s]
class MotionControlOtto:
  _MAP = (0, 1, 2)
  _SCALE = 1.0
  @staticmethod
  def move(speed, angle):
    s = float(speed) * MotionControlOtto._SCALE
    rad = math.radians(float(angle))
    vx = -s * math.sin(rad)
    vy = s * math.cos(rad)
    v = [vx, 0.5 * vx - 0.866 * vy, 0.5 * vx + 0.866 * vy]
    m = MotionControlOtto._MAP
    return [v[m[0]], v[m[1]], v[m[2]]]
  @staticmethod
  def move_forward(speed):
    return MotionControlOtto.move(speed, 0.0)
  @staticmethod
  def move_side(speed):
    return MotionControlOtto.move(speed, -90.0)
  @staticmethod
  def move_with_spin(speed, angle, spin):
    s = float(speed) * MotionControlOtto._SCALE
    rad = math.radians(float(angle))
    vx = -s * math.sin(rad)
    vy = s * math.cos(rad)
    sp = float(spin)
    v = [-vx + sp, 0.5 * vx - 0.866 * vy + sp, 0.5 * vx + 0.866 * vy + sp]
    m = MotionControlOtto._MAP
    return [v[m[0]], v[m[1]], v[m[2]]]
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
      self._d0 = max(-100.0, min(100.0, float(duties[0])))
      self._d1 = max(-100.0, min(100.0, float(duties[1])))
      self._d2 = max(-100.0, min(100.0, float(duties[2])))
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
class HeadingPID:
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
def wrap_deg(a):
  while a > 180.0:
    a -= 360.0
  while a < -180.0:
    a += 360.0
  return a
def select_target(detections, cfg, allow=None, target_class=None):
  if not detections:
    return None
  tc = int(cfg.tracking_target_class) if target_class is None else int(target_class)
  mc = cfg.tracking_min_confidence
  best = None
  best_preferred = False
  best_area = -1.0
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
    preferred = cid == tc or tc == 7
    area = d[8]
    if (best is None or
        (preferred and not best_preferred) or
        (preferred == best_preferred and area > best_area)):
      best = d
      best_preferred = preferred
      best_area = area
  return best
