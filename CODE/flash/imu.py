import math
from seekfree import IMU963RX
def _acc_to_g(raw):
  return raw / 4096.0
def _gyro_to_radps(raw, lsb):
  return raw / lsb * 0.01745329
class MadgwickAHRS:
  def __init__(self, beta=0.05, sample_freq=100.0):
    self.beta = beta
    self.dt = 1.0 / sample_freq
    self.q0 = 1.0
    self.q1 = 0.0
    self.q2 = 0.0
    self.q3 = 0.0
  def update(self, gx, gy, gz, ax, ay, az):
    q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
    acc_norm = math.sqrt(ax * ax + ay * ay + az * az)
    if acc_norm < 1e-6:
      self._integrate_gyro_only(gx, gy, gz)
      return
    ax /= acc_norm
    ay /= acc_norm
    az /= acc_norm
    _2q0 = 2.0 * q0
    _2q1 = 2.0 * q1
    _2q2 = 2.0 * q2
    _2q3 = 2.0 * q3
    f0 = _2q1 * q3 - _2q0 * q2 - ax
    f1 = _2q0 * q1 + _2q2 * q3 - ay
    f2 = 1.0 - _2q1 * q1 - _2q2 * q2 - az
    g0 = -_2q2 * f0 + _2q1 * f1
    g1 = _2q3 * f0 + _2q0 * f1 - 4.0 * q1 * f2
    g2 = -_2q0 * f0 + _2q3 * f1 - 4.0 * q2 * f2
    g3 = _2q1 * f0 + _2q2 * f1
    g_norm = math.sqrt(g0 * g0 + g1 * g1 + g2 * g2 + g3 * g3)
    if g_norm > 1e-10:
      g0 /= g_norm
      g1 /= g_norm
      g2 /= g_norm
      g3 /= g_norm
    qDot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
    qDot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
    qDot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
    qDot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
    beta = self.beta
    self.q0 += (qDot0 - beta * g0) * self.dt
    self.q1 += (qDot1 - beta * g1) * self.dt
    self.q2 += (qDot2 - beta * g2) * self.dt
    self.q3 += (qDot3 - beta * g3) * self.dt
    q_norm = math.sqrt(
      self.q0 * self.q0 + self.q1 * self.q1 +
      self.q2 * self.q2 + self.q3 * self.q3
    )
    if q_norm > 1e-10:
      self.q0 /= q_norm
      self.q1 /= q_norm
      self.q2 /= q_norm
      self.q3 /= q_norm
  def _integrate_gyro_only(self, gx, gy, gz):
    q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
    self.q0 += 0.5 * (-q1 * gx - q2 * gy - q3 * gz) * self.dt
    self.q1 += 0.5 * (q0 * gx + q2 * gz - q3 * gy) * self.dt
    self.q2 += 0.5 * (q0 * gy - q1 * gz + q3 * gx) * self.dt
    self.q3 += 0.5 * (q0 * gz + q1 * gy - q2 * gx) * self.dt
    q_norm = math.sqrt(
      self.q0 * self.q0 + self.q1 * self.q1 +
      self.q2 * self.q2 + self.q3 * self.q3
    )
    if q_norm > 1e-10:
      self.q0 /= q_norm
      self.q1 /= q_norm
      self.q2 /= q_norm
      self.q3 /= q_norm
  def yaw_deg(self):
    yaw = math.atan2(2.0 * (self.q0 * self.q3 + self.q1 * self.q2),
                     1.0 - 2.0 * (self.q2 * self.q2 + self.q3 * self.q3))
    return yaw * 57.29578
  def reset(self):
    self.q0 = 1.0
    self.q1 = 0.0
    self.q2 = 0.0
    self.q3 = 0.0
class ImuSensor:
  def __init__(self, calibrate_samples=100, beta=0.05, model="963"):
    if str(model) != "963":
      raise ValueError("only IMU963 is supported")
    self.model = "963"
    try:
      self.raw = IMU963RX(imu_type=IMU963RX.TYPE_RA)
    except (TypeError, AttributeError):
      self.raw = IMU963RX()
    self._gyro_lsb = 14.286
    self._acc_scale = 1.0 / 4096.0
    self._gyro_rad_scale = 0.01745329 / self._gyro_lsb
    self.data = self.raw.get()
    self._filter = MadgwickAHRS(beta=beta, sample_freq=200.0)
    self._bias = [0.0, 0.0, 0.0]
    self._calib_samples = calibrate_samples
    self._calib_count = 0
    self._calib_gx = 0.0
    self._calib_gy = 0.0
    self._calib_gz = 0.0
    self._calibrated = False
    self._snap = [0.0] * 8
    self._snap_idx = 0
    self._bias_alpha = 0.002
    self._still_count = 0
    self._still_needed = 100
    self._gyro_still = 0.0175
    self._acc_still = 0.05
    self._mag_enabled = False
    self._mag_alpha = 0.002
    self._mag_dead = 2.2
    self._mag_pull_max = 6.7
    self._mag_still_need = 100
    self._mag_lpf_alpha = 0.01
    self._mag_off = [0.0, 0.0, 0.0]
    self._mx = 0.0; self._my = 0.0; self._mz = 0.0
    self._fused_offset = 0.0
    self._mag_ref = None
    self._mag_rel_lpf = None
    self._gyro_dps = 0.0
    self._motor_on = False
    self._gyro_scale = 1.0
    self._spin_beta = 0.01
    self._spin_dps = 40.0
    self._spin_active = False
    self._resting_beta = beta
  def update(self):
    d = self.data
    ax_raw, ay_raw, az_raw = d[0], d[1], d[2]
    gx_raw, gy_raw, gz_raw = d[3], d[4], d[5]
    _mx = d[6] if len(d) >= 9 else 0
    _my = d[7] if len(d) >= 9 else 0
    _mz = d[8] if len(d) >= 9 else 0
    acc_scale = self._acc_scale
    gyro_scale = self._gyro_rad_scale
    ax = ax_raw * acc_scale
    ay = ay_raw * acc_scale
    az = az_raw * acc_scale
    gx = gx_raw * gyro_scale
    gy = gy_raw * gyro_scale
    gz = gz_raw * gyro_scale
    if not self._calibrated:
      self._calib_gx += gx
      self._calib_gy += gy
      self._calib_gz += gz
      self._calib_count += 1
      if self._calib_count >= self._calib_samples:
        n = float(self._calib_count)
        self._bias[0] = self._calib_gx / n
        self._bias[1] = self._calib_gy / n
        self._bias[2] = self._calib_gz / n
        self._calibrated = True
        self._filter.reset()
        self._fused_offset = 0.0
        self._mag_ref = None
        self._mag_rel_lpf = None
      return
    gx -= self._bias[0]
    gy -= self._bias[1]
    gz -= self._bias[2]
    gyro_mag = math.sqrt(gx * gx + gy * gy + gz * gz)
    self._gyro_dps = gyro_mag * 57.29578
    acc_mag = math.sqrt(ax * ax + ay * ay + az * az)
    is_still = (gyro_mag < self._gyro_still) and (abs(acc_mag - 1.0) < self._acc_still)
    if is_still:
      self._still_count += 1
      if self._still_count >= self._still_needed:
        a = self._bias_alpha
        self._bias[0] += a * gx
        self._bias[1] += a * gy
        self._bias[2] += a * gz
    else:
      self._still_count = 0
    s = self._gyro_scale
    gx_f = gx * s
    gy_f = gy * s
    gz_f = gz * s
    if self._gyro_dps >= self._spin_dps:
      if not self._spin_active:
        self._resting_beta = self._filter.beta
        self._spin_active = True
      if self._spin_beta < self._filter.beta:
        self._filter.beta = self._spin_beta
    elif self._gyro_dps < self._spin_dps * 0.5:
      if self._spin_active:
        self._filter.beta = self._resting_beta
        self._spin_active = False
    self._filter.update(gx_f, gy_f, gz_f, ax, ay, az)
    self._mx = _mx - self._mag_off[0]
    self._my = _my - self._mag_off[1]
    self._mz = _mz - self._mag_off[2]
    if self._mag_enabled:
      if (self._still_count >= self._mag_still_need
          and not self._motor_on and self._gyro_dps < 0.5):
        mag = self._mag_heading_from_q(
          self._filter.q0, self._filter.q1, self._filter.q2, self._filter.q3)
        if mag is not None:
          mad = self._filter.yaw_deg()
          prev = self._normalize_angle(mad + self._fused_offset)
          if self._mag_ref is None:
            self._mag_ref = self._normalize_angle(mag - prev)
            self._mag_rel_lpf = 0.0
          mag_rel = self._normalize_angle(mag - self._mag_ref)
          if self._mag_rel_lpf is None:
            self._mag_rel_lpf = mag_rel
          else:
            self._mag_rel_lpf = self._normalize_angle(
              self._mag_rel_lpf + self._mag_lpf_alpha * self._normalize_angle(
                mag_rel - self._mag_rel_lpf))
          diff = self._normalize_angle(self._mag_rel_lpf - prev)
          ad = abs(diff)
          if ad >= self._mag_dead:
            if ad > self._mag_pull_max:
              diff = self._mag_pull_max if diff > 0 else -self._mag_pull_max
            self._fused_offset += self._mag_alpha * diff
    f = self._filter
    off = self._snap_idx * 4
    self._snap[off]     = f.q0
    self._snap[off + 1] = f.q1
    self._snap[off + 2] = f.q2
    self._snap[off + 3] = f.q3
    self._snap_idx ^= 1
  def _read_snap(self):
    off = (1 - self._snap_idx) * 4
    snap = self._snap
    return snap[off], snap[off + 1], snap[off + 2], snap[off + 3]
  @staticmethod
  def _yaw_from_quat(q0, q1, q2, q3):
    yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2),
                     1.0 - 2.0 * (q2 * q2 + q3 * q3))
    return yaw * 57.29578
  def get_madgwick_yaw(self):
    if not self._calibrated:
      return 0.0
    off = (1 - self._snap_idx) * 4
    snap = self._snap
    return self._yaw_from_quat(
      snap[off], snap[off + 1], snap[off + 2], snap[off + 3])
  def get_yaw(self, motor_on=False):
    if not self._calibrated:
      return 0.0
    if self._mag_enabled:
      yaw, _ = self.get_fused_yaw(motor_on=motor_on, apply=False)
      return yaw
    return self.get_madgwick_yaw()
  def get_mag_heading(self):
    if not self._mag_enabled or not self._calibrated:
      return None
    q0, q1, q2, q3 = self._read_snap()
    return self._mag_heading_from_q(q0, q1, q2, q3)
  def _mag_heading_from_q(self, q0, q1, q2, q3):
    mx, my, mz = self._mx, self._my, self._mz
    if abs(mx) < 1 and abs(my) < 1:
      return None
    sin_pitch = 2.0 * (q0 * q1 - q2 * q3)
    if abs(sin_pitch) > 1.0:
      sin_pitch = 1.0 if sin_pitch > 0 else -1.0
    pitch = math.asin(sin_pitch)
    roll = math.atan2(2.0 * (q0 * q2 + q1 * q3),
                      1.0 - 2.0 * (q1 * q1 + q2 * q2))
    cos_r, sin_r = math.cos(roll), math.sin(roll)
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)
    mx_h = mx * cos_p + mz * sin_p
    my_h = mx * sin_r * sin_p + my * cos_r - mz * sin_r * cos_p
    return self._normalize_angle(math.atan2(-my_h, mx_h) * 57.29578)
  def get_mag_rel(self):
    mag = self.get_mag_heading()
    if mag is None or self._mag_ref is None:
      return None
    return self._normalize_angle(mag - self._mag_ref)
  def get_fused_yaw(self, motor_on=False, alpha=None, apply=False):
    base = self.get_madgwick_yaw()
    fused = self._normalize_angle(base + self._fused_offset)
    if not self._mag_enabled:
      return fused, "mad"
    if motor_on or self._motor_on or self._gyro_dps >= 0.5:
      return fused, "mad"
    mag_rel = self.get_mag_rel()
    if mag_rel is None:
      return fused, "mad"
    diff = self._normalize_angle(mag_rel - fused)
    ad = abs(diff)
    if ad < self._mag_dead:
      return fused, "mad"
    if self._still_count < self._mag_still_need:
      return fused, "mad"
    if apply:
      a = self._mag_alpha if alpha is None else alpha
      if a > 0.0:
        corr = diff
        if ad > self._mag_pull_max:
          corr = self._mag_pull_max if diff > 0 else -self._mag_pull_max
        self._fused_offset += a * corr
        fused = self._normalize_angle(base + self._fused_offset)
    return fused, "fused"
  @property
  def mag_enabled(self):
    return self._mag_enabled
  @mag_enabled.setter
  def mag_enabled(self, v):
    was_on = self._mag_enabled
    self._mag_enabled = bool(v)
    if self._mag_enabled and not was_on:
      self._mag_ref = None
      self._mag_rel_lpf = None
    elif not self._mag_enabled:
      self._fused_offset = 0.0
      self._mag_ref = None
      self._mag_rel_lpf = None
  @property
  def mag_data(self):
    return (self._mx, self._my, self._mz)
  @property
  def mag_ready(self):
    return (self._mag_enabled and self._mag_ref is not None and
            self._still_count >= self._mag_still_need)
  @property
  def fused_offset(self):
    return self._fused_offset
  @property
  def still_count(self):
    return self._still_count
  def set_mag_offset(self, mx_off, my_off, mz_off=0.0):
    self._mag_off = [mx_off, my_off, mz_off]
  def set_mag_alpha(self, alpha):
    self._mag_alpha = max(0.0, min(0.1, alpha))
  def set_fusion_params(self, gyro_still=0.0175, acc_still=0.05,
                         bias_alpha=0.002, mag_alpha=0.002,
                         mag_dead=2.2, mag_pull_max=6.7, mag_still_need=100,
                         still_needed=100, mag_lpf_alpha=0.01,
                         gyro_scale=1.0, spin_beta=0.01, spin_dps=40.0):
    self._gyro_still = float(gyro_still)
    self._acc_still = float(acc_still)
    self._bias_alpha = float(bias_alpha)
    self._mag_alpha = float(mag_alpha)
    self._mag_dead = float(mag_dead)
    self._mag_pull_max = float(mag_pull_max)
    self._mag_still_need = int(mag_still_need)
    self._still_needed = int(still_needed)
    self._mag_lpf_alpha = float(mag_lpf_alpha)
    self._gyro_scale = float(gyro_scale)
    self._spin_beta = float(spin_beta)
    self._spin_dps = float(spin_dps)
  @staticmethod
  def _normalize_angle(a):
    while a > 180.0: a -= 360.0
    while a < -180.0: a += 360.0
    return a
  @property
  def is_calibrated(self):
    return self._calibrated
