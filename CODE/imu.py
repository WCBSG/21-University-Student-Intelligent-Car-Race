"""
imu.py — 陀螺仪姿态融合系统

支持:
  - IMU963RA (seekfree.IMU963RX) — 9 轴，默认；磁力计暂不参与融合
  - IMU660RX (LSM6DSO 6 轴) — model="660"

融合: Madgwick 6 轴 AHRS（加速度钉 Roll/Pitch，Yaw 靠陀螺+零偏）
963 加速度标称 ~52Hz、陀螺 ~208Hz；ticker 仍 100Hz 跑融合（加速度帧间复用）。
陀螺 LSB：660=16.384，963(LSM6DSR)=14.286。

用法:
  imu = ImuSensor(model="963")   # 或 "660"
  tkr.capture_list(imu.raw)
  # ticker 回调: imu.update()
  yaw = imu.get_yaw()
"""

import math
from seekfree import IMU660RX, IMU963RX

# =============================================================================
#                          物理量纲转换
# =============================================================================
# ±8g → 4096 LSB/g（两边一致）
# 陀螺 ±2000dps 灵敏度因芯片不同：
#   660 实测好用 16.384（ICM 系常见）
#   963 = LSM6DSR → 70 mdps/LSB = 14.286 LSB/dps
ACC_LSB_PER_G = 4096.0
GYRO_LSB_660 = 16.384
GYRO_LSB_963 = 14.286
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


def _acc_to_g(raw):
  return raw / ACC_LSB_PER_G


def _gyro_to_radps(raw, lsb):
  return raw / lsb * DEG_TO_RAD


def _gyro_to_dps(raw, lsb):
  return raw / lsb


# =============================================================================
#                       磁力计硬铁标定 (min/max)
# =============================================================================

class MagCalib:
  """
  水平转圈采集 mx/my(/mz) 极值，中心即硬铁偏移。
  菜单里反复 feed(raw)；完成后 offset → set_mag_offset + 存 config。
  """

  def __init__(self):
    self.reset()

  def reset(self):
    self.mx_min = 1e9
    self.mx_max = -1e9
    self.my_min = 1e9
    self.my_max = -1e9
    self.mz_min = 1e9
    self.mz_max = -1e9
    self.n = 0

  def feed(self, mx, my, mz=0.0):
    if mx < self.mx_min:
      self.mx_min = mx
    if mx > self.mx_max:
      self.mx_max = mx
    if my < self.my_min:
      self.my_min = my
    if my > self.my_max:
      self.my_max = my
    if mz < self.mz_min:
      self.mz_min = mz
    if mz > self.mz_max:
      self.mz_max = mz
    self.n += 1

  @property
  def span_xy(self):
    return (self.mx_max - self.mx_min, self.my_max - self.my_min)

  @property
  def ready(self):
    """水平面有足够椭圆跨度才认为转过一圈。"""
    dx, dy = self.span_xy
    return self.n >= 50 and dx > 80.0 and dy > 80.0

  @property
  def offset(self):
    return (
      (self.mx_max + self.mx_min) * 0.5,
      (self.my_max + self.my_min) * 0.5,
      (self.mz_max + self.mz_min) * 0.5,
    )


# =============================================================================
#                       Madgwick AHRS 滤波器
# =============================================================================

class MadgwickAHRS:
  """
  Madgwick 梯度下降法姿态融合 (6 轴版本，无磁力计)。

  参考：S. Madgwick, "An efficient orientation filter for IMU and MARG arrays"

  参数：
    beta        — 算法增益 (rad/s)，越大收敛越快但噪声越大
    sample_freq — 采样频率 (Hz)
  """

  def __init__(self, beta=0.05, sample_freq=100.0):
    self.beta = beta
    self.dt = 1.0 / sample_freq

    # 四元数 [w, x, y, z] — 初始化为水平朝前
    self.q0 = 1.0
    self.q1 = 0.0
    self.q2 = 0.0
    self.q3 = 0.0

  # ——————————————————————————————————————————————————————————
  #                      核心更新
  # ——————————————————————————————————————————————————————————

  def update(self, gx, gy, gz, ax, ay, az):
    """
    输入：
      gx, gy, gz — 陀螺仪角速度 (rad/s)
      ax, ay, az — 加速度计值 (g 单位)

    每帧调用一次，内部积分 dt。
    """
    q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3

    # —— 加速度计归一化 ————————————————————————————————
    acc_norm = math.sqrt(ax * ax + ay * ay + az * az)
    if acc_norm < 1e-6:
      # 加速度计数据异常，仅靠陀螺仪积分
      self._integrate_gyro_only(gx, gy, gz)
      return

    ax /= acc_norm
    ay /= acc_norm
    az /= acc_norm

    # —— 梯度下降 (目标函数: 重力 [0,0,1] 旋转到传感器系) ——

    # 目标函数 f = R(q)^T · g_earth - a_measured
    #   f0 = 2*(q1*q3 - q0*q2) - ax
    #   f1 = 2*(q0*q1 + q2*q3) - ay
    #   f2 = 2*(0.5 - q1² - q2²) - az
    _2q0 = 2.0 * q0
    _2q1 = 2.0 * q1
    _2q2 = 2.0 * q2
    _2q3 = 2.0 * q3

    f0 = _2q1 * q3 - _2q0 * q2 - ax
    f1 = _2q0 * q1 + _2q2 * q3 - ay
    f2 = 1.0 - _2q1 * q1 - _2q2 * q2 - az

    # 雅可比矩阵 J (3×4) 的转置 J^T × f → 梯度 (4×1)
    # J^T 各列:
    #   col0: [-2q2,  2q1,  0  ]
    #   col1: [ 2q3,  2q0, -4q1]
    #   col2: [-2q0,  2q3, -4q2]
    #   col3: [ 2q1,  2q2,  0  ]
    g0 = -_2q2 * f0 + _2q1 * f1
    g1 = _2q3 * f0 + _2q0 * f1 - 4.0 * q1 * f2
    g2 = -_2q0 * f0 + _2q3 * f1 - 4.0 * q2 * f2
    g3 = _2q1 * f0 + _2q2 * f1

    # 梯度归一化
    g_norm = math.sqrt(g0 * g0 + g1 * g1 + g2 * g2 + g3 * g3)
    if g_norm > 1e-10:
      g0 /= g_norm
      g1 /= g_norm
      g2 /= g_norm
      g3 /= g_norm

    # —— 陀螺仪四元数导数 ——————————————————————————————
    # qDot_ω = 0.5 * q ⊗ [0, ω]
    qDot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
    qDot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
    qDot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
    qDot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

    # —— 融合：qDot = qDot_ω - β · ∇f —————————————————
    beta = self.beta
    self.q0 += (qDot0 - beta * g0) * self.dt
    self.q1 += (qDot1 - beta * g1) * self.dt
    self.q2 += (qDot2 - beta * g2) * self.dt
    self.q3 += (qDot3 - beta * g3) * self.dt

    # —— 四元数归一化 ——————————————————————————————————
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
    """加速度计无效时仅靠陀螺仪积分。★ 需要归一化防止四元数幅度漂移。"""
    q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
    self.q0 += 0.5 * (-q1 * gx - q2 * gy - q3 * gz) * self.dt
    self.q1 += 0.5 * (q0 * gx + q2 * gz - q3 * gy) * self.dt
    self.q2 += 0.5 * (q0 * gy - q1 * gz + q3 * gx) * self.dt
    self.q3 += 0.5 * (q0 * gz + q1 * gy - q2 * gx) * self.dt

    # 归一化（防止连续多帧仅靠陀螺仪积分导致幅度漂移）
    q_norm = math.sqrt(
      self.q0 * self.q0 + self.q1 * self.q1 +
      self.q2 * self.q2 + self.q3 * self.q3
    )
    if q_norm > 1e-10:
      self.q0 /= q_norm
      self.q1 /= q_norm
      self.q2 /= q_norm
      self.q3 /= q_norm

  # ——————————————————————————————————————————————————————————
  #                      欧拉角提取
  # ——————————————————————————————————————————————————————————

  def get_yaw(self):
    """
    偏航角 (绕 Z 轴), 单位 deg, 范围 [-180, 180]。
    正值 = 逆时针 (左转)，0 = 初始朝向。
    """
    q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
    yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2),
                     1.0 - 2.0 * (q2 * q2 + q3 * q3))
    return yaw * RAD_TO_DEG

  def get_pitch(self):
    """俯仰角, deg。正值 = 抬头。"""
    q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
    sin_pitch = 2.0 * (q0 * q1 - q2 * q3)
    if abs(sin_pitch) > 1.0:
      sin_pitch = 1.0 if sin_pitch > 0 else -1.0
    return math.asin(sin_pitch) * RAD_TO_DEG

  def get_roll(self):
    """滚转角, deg。正值 = 右倾。"""
    q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
    roll = math.atan2(2.0 * (q0 * q2 + q1 * q3),
                      1.0 - 2.0 * (q1 * q1 + q2 * q2))
    return roll * RAD_TO_DEG

  def reset(self):
    """重置滤波器状态 (例如重新标定后)。"""
    self.q0 = 1.0
    self.q1 = 0.0
    self.q2 = 0.0
    self.q3 = 0.0


# =============================================================================
#                         IMU660RX 传感器封装
# =============================================================================

class ImuSensor:
  """
  IMU + Madgwick 姿态融合。

  model:
    "963" — IMU963RX / IMU963RA（get 含 mag，融合仍用前 6 轴）
    "660" — IMU660RX
  """

  def __init__(self, calibrate_samples=100, beta=0.05, model="963"):
    """
    calibrate_samples — 启动静止标定帧数 (~1s @100Hz)
    beta              — Madgwick 增益 (实测 0.05 较稳)
    model             — "963" | "660"
    """
    self.model = model
    if model == "660":
      self.raw = IMU660RX()
      self._gyro_lsb = GYRO_LSB_660
    else:
      # TYPE_RA 与 help 一致；AUTO 亦可
      try:
        self.raw = IMU963RX(imu_type=IMU963RX.TYPE_RA)
      except (TypeError, AttributeError):
        self.raw = IMU963RX()
      self._gyro_lsb = GYRO_LSB_963

    self.data = self.raw.get()  # 链接缓冲区（963 为 9 元，660 为 6 元）

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

    # 磁力计 (仅 963)
    self._mag_enabled = False      # MATCH 默认关
    self._mag_alpha = 0.01         # 互补滤波系数
    self._mag_off = [0.0, 0.0, 0.0]  # 硬铁偏移
    self._mx = 0.0; self._my = 0.0; self._mz = 0.0
    self._gyro_yaw = 0.0           # 纯陀螺累积 yaw (deg), ISR 写
    self._fused_offset = 0.0       # mag 修正累积 (deg), 主循环写
    self._gyro_dps = 0.0           # 陀螺模长 (deg/s), 用于 ω 门控

  def update(self):
    """ticker 回调：标定 / 去偏 / Madgwick / 快照。"""
    d = self.data
    # 963: [ax,ay,az,gx,gy,gz,mx,my,mz]；660: 无 mag — 统一取前 6
    ax_raw, ay_raw, az_raw = d[0], d[1], d[2]
    gx_raw, gy_raw, gz_raw = d[3], d[4], d[5]

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
      return

    gx -= self._bias[0]
    gy -= self._bias[1]
    gz -= self._bias[2]

    # ★ 660 轴 remap → 对齐 963 车体系 (X前 Y右 Z上)
    # 实测: 660 前倾→roll↓, 左倾→pitch↑ → ax,ay 互换 + Y 取反
    if self.model == "660":
      ax, ay = ay, -ax
      gx, gy = gy, -gx

    gyro_mag = math.sqrt(gx * gx + gy * gy + gz * gz)
    self._gyro_dps = gyro_mag * RAD_TO_DEG  # ω 门控用
    acc_mag = math.sqrt(ax * ax + ay * ay + az * az)
    is_still = (gyro_mag < 0.0175) and (abs(acc_mag - 1.0) < 0.05)

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

    # 磁力计快照 (仅 963；始终更新，便于标定/菜单；融合仍看 mag_enabled)
    if self.model == "963":
      self._mx = d[6] - self._mag_off[0]
      self._my = d[7] - self._mag_off[1]
      self._mz = d[8] - self._mag_off[2]

    # 纯陀螺 yaw 累积 (用于互补融合)
    self._gyro_yaw += gz * self._filter.dt * RAD_TO_DEG
    self._gyro_yaw = self._normalize_angle(self._gyro_yaw)

    f = self._filter
    off = self._snap_idx * 5  # 每槽 5 个 float
    self._snap[off]     = f.q0
    self._snap[off + 1] = f.q1
    self._snap[off + 2] = f.q2
    self._snap[off + 3] = f.q3
    self._snap[off + 4] = self._gyro_yaw
    self._snap_idx ^= 1

  # ——————————————————————————————————————————————————————————
  #                      姿态获取（读快照）
  # ——————————————————————————————————————————————————————————

  def _read_snap(self):
    off = (1 - self._snap_idx) * 5
    return tuple(self._snap[off + i] for i in range(5))

  def get_yaw(self, motor_on=False):
    """
    偏航角, deg, [-180, 180]。
    mag_enabled → 互补融合; 否则 Madgwick 纯陀螺。
    motor_on: 电机转时传 True, 跳过磁修正。
    """
    if not self._calibrated:
      return 0.0
    if self._mag_enabled:
      yaw, _ = self.get_fused_yaw(motor_on=motor_on)
      return yaw
    q0, q1, q2, q3, _ = self._read_snap()
    yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2),
                     1.0 - 2.0 * (q2 * q2 + q3 * q3))
    return yaw * RAD_TO_DEG

  def get_pitch(self):
    if not self._calibrated:
      return 0.0
    q0, q1, q2, q3, _ = self._read_snap()
    sin_pitch = 2.0 * (q0 * q1 - q2 * q3)
    if abs(sin_pitch) > 1.0:
      sin_pitch = 1.0 if sin_pitch > 0 else -1.0
    return math.asin(sin_pitch) * RAD_TO_DEG

  def get_roll(self):
    if not self._calibrated:
      return 0.0
    q0, q1, q2, q3, _ = self._read_snap()
    roll = math.atan2(2.0 * (q0 * q2 + q1 * q3),
                      1.0 - 2.0 * (q1 * q1 + q2 * q2))
    return roll * RAD_TO_DEG

  def get_gyro_yaw(self):
    """纯陀螺累积 yaw (deg)。"""
    if not self._calibrated:
      return 0.0
    return self._snap[(1 - self._snap_idx) * 5 + 4]

  def get_mag_heading(self):
    """
    磁力计航向角 (deg), 倾角补偿, 车体系 (X前 Y右 Z上)。
    仅 963 有效；660 或无 mag 数据返回 None。
    """
    if self.model != "963" or not self._mag_enabled:
      return None
    if not self._calibrated:
      return None
    mx, my, mz = self._mx, self._my, self._mz
    if abs(mx) < 1 and abs(my) < 1:
      return None  # mag 数据太弱

    roll = self.get_roll() * DEG_TO_RAD
    pitch = self.get_pitch() * DEG_TO_RAD
    cos_r, sin_r = math.cos(roll), math.sin(roll)
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)

    # 倾角补偿：把磁矢量投到水平面
    mx_h = mx * cos_p + mz * sin_p
    my_h = mx * sin_r * sin_p + my * cos_r - mz * sin_r * cos_p

    heading = math.atan2(my_h, mx_h) * RAD_TO_DEG  # Y右系
    return self._normalize_angle(heading)

  def get_fused_yaw(self, motor_on=False, alpha=None):
    """
    互补融合航向角 (deg)。★ 写回 _fused_offset 使修正持续累积。
    motor_on 或 |ω|>5°/s → α=0; 静止/低速 → α 融合磁。
    返回 (yaw, source): source='gyro'|'mag'。
    """
    gyro = self.get_gyro_yaw()
    if not self._mag_enabled:
      return gyro, "gyro"
    if self.model not in ("660", "963"):
      return gyro, "gyro"

    mag = self.get_mag_heading()
    if mag is None:
      return gyro, "gyro"

    # 自适应 α
    if alpha is not None:
      a = alpha
    elif motor_on or self._gyro_dps > 5.0:  # ω 门控
      a = 0.0
    else:
      a = self._mag_alpha

    if a <= 0.0:
      return gyro + self._fused_offset, "gyro"

    # 互补: offset += α·(mag − fused_prev)
    prev = gyro + self._fused_offset
    diff = self._normalize_angle(mag - prev)
    self._fused_offset += a * diff
    fused = gyro + self._fused_offset
    return self._normalize_angle(fused), "mag"

  # ——————————————————————————————————————————————————————————
  #                      磁力计参数
  # ——————————————————————————————————————————————————————————

  @property
  def mag_enabled(self):
    return self._mag_enabled

  @mag_enabled.setter
  def mag_enabled(self, v):
    if not bool(v):
      self._fused_offset = 0.0   # 关 mag 时清修正
    self._mag_enabled = bool(v)

  @property
  def mag_data(self):
    """(mx, my, mz) raw（已去硬铁偏移）。"""
    return (self._mx, self._my, self._mz)

  def set_mag_offset(self, mx_off, my_off, mz_off=0.0):
    self._mag_off = [mx_off, my_off, mz_off]

  def set_mag_alpha(self, alpha):
    self._mag_alpha = max(0.0, min(0.1, alpha))

  # ——————————————————————————————————————————————————————————
  #                      工具
  # ——————————————————————————————————————————————————————————

  @staticmethod
  def _normalize_angle(a):
    while a > 180.0: a -= 360.0
    while a < -180.0: a += 360.0
    return a

  # ——————————————————————————————————————————————————————————
  #                      原始数据
  # ——————————————————————————————————————————————————————————

  def get_gyro_dps(self):
    """陀螺仪 deg/s (已去零偏, 660 已 remap)。"""
    raw = self.data
    bx, by, bz = self._bias
    lsb = self._gyro_lsb
    gx = _gyro_to_dps(raw[3], lsb) - bx * RAD_TO_DEG
    gy = _gyro_to_dps(raw[4], lsb) - by * RAD_TO_DEG
    gz = _gyro_to_dps(raw[5], lsb) - bz * RAD_TO_DEG
    if self.model == "660":
      gx, gy = gy, -gx  # 同 update() remap
    return (gx, gy, gz)

  def get_accel_g(self):
    """加速度计 g 值 (660 已 remap)。"""
    raw = self.data
    ax = _acc_to_g(raw[0])
    ay = _acc_to_g(raw[1])
    az = _acc_to_g(raw[2])
    if self.model == "660":
      ax, ay = ay, -ax  # 同 update() remap
    return (ax, ay, az)

  @property
  def is_calibrated(self):
    return self._calibrated

  @property
  def is_still(self):
    """当前是否判定为静止 (连续 0.5s 满足条件)。"""
    return self._still_count >= self._still_needed

  @property
  def bias_dps(self):
    """零偏, deg/s。"""
    return (self._bias[0] * RAD_TO_DEG,
            self._bias[1] * RAD_TO_DEG,
            self._bias[2] * RAD_TO_DEG)

  # ——————————————————————————————————————————————————————————
  #                      手动标定
  # ——————————————————————————————————————————————————————————

  def recalibrate(self):
    """
    手动触发陀螺仪零偏重标定。
    标定期间机器人须静止 1 秒，完成后自动恢复滤波器运行。
    可在代码中任意位置调用，也可通过菜单触发。
    """
    self._calib_count = 0
    self._calib_gx = 0.0
    self._calib_gy = 0.0
    self._calib_gz = 0.0
    self._still_count = 0
    self._gyro_yaw = 0.0
    self._fused_offset = 0.0
    self._calibrated = False