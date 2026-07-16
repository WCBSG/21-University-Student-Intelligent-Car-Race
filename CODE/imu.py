"""
imu.py — IMU963 + Madgwick 姿态融合（比赛固件）

融合: Madgwick 6 轴 AHRS（加速度钉 Roll/Pitch，Yaw 靠陀螺+零偏）
可选静止磁慢纠（相对标定零点，非磁北）。
ticker @100Hz: imu.update()
"""

import math
from seekfree import IMU963RX

# ±8g → 4096 LSB/g；963 = LSM6DSR → 70 mdps/LSB = 14.286 LSB/dps
ACC_LSB_PER_G = 4096.0
GYRO_LSB_963 = 14.286
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


def _acc_to_g(raw):
  return raw / ACC_LSB_PER_G


def _gyro_to_radps(raw, lsb):
  return raw / lsb * DEG_TO_RAD


# =============================================================================
#                       Madgwick AHRS 滤波器
# =============================================================================

class MadgwickAHRS:
  """Madgwick 梯度下降法姿态融合 (6 轴，无磁力计)。"""

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

  def reset(self):
    self.q0 = 1.0
    self.q1 = 0.0
    self.q2 = 0.0
    self.q3 = 0.0


# =============================================================================
#                         ImuSensor（963）
# =============================================================================

class ImuSensor:
  """IMU963 + Madgwick；可选磁相对慢纠。"""

  def __init__(self, calibrate_samples=100, beta=0.05, model="963"):
    self.model = "963"
    try:
      self.raw = IMU963RX(imu_type=IMU963RX.TYPE_RA)
    except (TypeError, AttributeError):
      self.raw = IMU963RX()
    self._gyro_lsb = GYRO_LSB_963

    self.data = self.raw.get()

    self._filter = MadgwickAHRS(beta=beta, sample_freq=100.0)

    self._bias = [0.0, 0.0, 0.0]

    self._calib_samples = calibrate_samples
    self._calib_count = 0
    self._calib_gx = 0.0
    self._calib_gy = 0.0
    self._calib_gz = 0.0
    self._calibrated = False

    self._snap = [0.0] * 10  # 2×(q0,q1,q2,q3,gyro_yaw)
    self._snap_idx = 0

    self._bias_alpha = 0.002
    self._still_count = 0
    self._still_needed = 50
    self._gyro_still = 0.0175
    self._acc_still = 0.05

    self._mag_enabled = False
    self._mag_alpha = 0.002
    self._mag_dead = 2.2
    self._mag_pull_max = 6.7
    self._mag_still_need = 50
    self._mag_off = [0.0, 0.0, 0.0]
    self._mx = 0.0; self._my = 0.0; self._mz = 0.0
    self._gyro_yaw = 0.0
    self._fused_offset = 0.0
    self._mag_ref = None
    self._mag_rel_lpf = None
    self._gyro_dps = 0.0
    self._motor_on = False

  def update(self):
    """ticker 回调：标定 / 去偏 / Madgwick / 快照。"""
    d = self.data
    ax_raw, ay_raw, az_raw = d[0], d[1], d[2]
    gx_raw, gy_raw, gz_raw = d[3], d[4], d[5]
    _mx = d[6] if len(d) >= 9 else 0
    _my = d[7] if len(d) >= 9 else 0
    _mz = d[8] if len(d) >= 9 else 0

    ax = _acc_to_g(ax_raw)
    ay = _acc_to_g(ay_raw)
    az = _acc_to_g(az_raw)
    lsb = self._gyro_lsb
    gx = _gyro_to_radps(gx_raw, lsb)
    gy = _gyro_to_radps(gy_raw, lsb)
    gz = _gyro_to_radps(gz_raw, lsb)

    if not self._calibrated:
      self._calib_gx += gx
      self._calib_gy += gy
      self._calib_gz += gz
      self._calib_count += 1

      if self._calib_count >= self._calib_samples:
        n = float(self._calib_count)
        self._bias = [self._calib_gx / n,
                      self._calib_gy / n,
                      self._calib_gz / n]
        self._calibrated = True
        self._filter.reset()
        self._gyro_yaw = 0.0
        self._fused_offset = 0.0
        self._mag_ref = None
        self._mag_rel_lpf = None
      return

    gx -= self._bias[0]
    gy -= self._bias[1]
    gz -= self._bias[2]

    gyro_mag = math.sqrt(gx * gx + gy * gy + gz * gz)
    self._gyro_dps = gyro_mag * RAD_TO_DEG
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

    self._filter.update(gx, gy, gz, ax, ay, az)

    self._mx = _mx - self._mag_off[0]
    self._my = _my - self._mag_off[1]
    self._mz = _mz - self._mag_off[2]

    self._gyro_yaw += gz * self._filter.dt * RAD_TO_DEG
    self._gyro_yaw = self._normalize_angle(self._gyro_yaw)

    if self._mag_enabled:
      if (self._still_count >= self._mag_still_need
          and not self._motor_on and self._gyro_dps < 0.5):
        mag = self._mag_heading_from_q(
          self._filter.q0, self._filter.q1, self._filter.q2, self._filter.q3)
        if mag is not None:
          prev = self._normalize_angle(self._gyro_yaw + self._fused_offset)
          if self._mag_ref is None:
            self._mag_ref = self._normalize_angle(mag - prev)
            self._mag_rel_lpf = 0.0
          mag_rel = self._normalize_angle(mag - self._mag_ref)
          if self._mag_rel_lpf is None:
            self._mag_rel_lpf = mag_rel
          else:
            self._mag_rel_lpf = self._normalize_angle(
              self._mag_rel_lpf + 0.04 * self._normalize_angle(
                mag_rel - self._mag_rel_lpf))
          diff = self._normalize_angle(self._mag_rel_lpf - prev)
          ad = abs(diff)
          if self._mag_dead <= ad <= self._mag_pull_max:
            self._fused_offset += self._mag_alpha * diff

    f = self._filter
    off = self._snap_idx * 5
    self._snap[off]     = f.q0
    self._snap[off + 1] = f.q1
    self._snap[off + 2] = f.q2
    self._snap[off + 3] = f.q3
    self._snap[off + 4] = self._gyro_yaw
    self._snap_idx ^= 1

  def _read_snap(self):
    off = (1 - self._snap_idx) * 5
    return tuple(self._snap[off + i] for i in range(5))

  def get_yaw(self, motor_on=False):
    if not self._calibrated:
      return 0.0
    if self._mag_enabled:
      yaw, _ = self.get_fused_yaw(motor_on=motor_on, apply=False)
      return yaw
    q0, q1, q2, q3, _ = self._read_snap()
    yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2),
                     1.0 - 2.0 * (q2 * q2 + q3 * q3))
    return yaw * RAD_TO_DEG

  def get_gyro_yaw(self):
    if not self._calibrated:
      return 0.0
    return self._snap[(1 - self._snap_idx) * 5 + 4]

  def get_mag_heading(self):
    if not self._mag_enabled or not self._calibrated:
      return None
    q0, q1, q2, q3, _ = self._read_snap()
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
    return self._normalize_angle(math.atan2(-my_h, mx_h) * RAD_TO_DEG)

  def get_mag_rel(self):
    mag = self.get_mag_heading()
    if mag is None or self._mag_ref is None:
      return None
    return self._normalize_angle(mag - self._mag_ref)

  def get_fused_yaw(self, motor_on=False, alpha=None, apply=False):
    gyro = self.get_gyro_yaw()
    fused = self._normalize_angle(gyro + self._fused_offset)
    if not self._mag_enabled:
      return fused, "gyro"
    if motor_on or self._motor_on or self._gyro_dps >= 0.5:
      return fused, "gyro"
    mag_rel = self.get_mag_rel()
    if mag_rel is None:
      return fused, "gyro"
    diff = self._normalize_angle(mag_rel - fused)
    ad = abs(diff)
    if ad < self._mag_dead or ad > self._mag_pull_max:
      return fused, "gyro"
    if self._still_count < self._mag_still_need:
      return fused, "gyro"
    if apply:
      a = self._mag_alpha if alpha is None else alpha
      if a > 0.0:
        self._fused_offset += a * diff
        fused = self._normalize_angle(gyro + self._fused_offset)
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

  def set_mag_offset(self, mx_off, my_off, mz_off=0.0):
    self._mag_off = [mx_off, my_off, mz_off]

  def set_mag_alpha(self, alpha):
    self._mag_alpha = max(0.0, min(0.1, alpha))

  def set_fusion_params(self, gyro_still=0.0175, acc_still=0.05,
                         bias_alpha=0.002, mag_alpha=0.002,
                         mag_dead=2.2, mag_pull_max=6.7, mag_still_need=50):
    self._gyro_still = float(gyro_still)
    self._acc_still = float(acc_still)
    self._bias_alpha = float(bias_alpha)
    self._mag_alpha = float(mag_alpha)
    self._mag_dead = float(mag_dead)
    self._mag_pull_max = float(mag_pull_max)
    self._mag_still_need = int(mag_still_need)

  @staticmethod
  def _normalize_angle(a):
    while a > 180.0: a -= 360.0
    while a < -180.0: a += 360.0
    return a

  @property
  def is_calibrated(self):
    return self._calibrated
