"""
imu.py — 陀螺仪姿态融合系统

使用 IMU660RX (LSM6DSO 6 轴) + Madgwick AHRS 滤波获取偏航角。

原理：
  - Madgwick 梯度下降法融合加速度计 (重力参考) + 陀螺仪 (角速度积分)
  - 加速度计纠正 Roll/Pitch 漂移，Yaw 无磁力计故会缓慢漂移 (约 1-3°/min)
  - 通过启动时陀螺仪零偏标定 + 自适应增益减缓漂移

用法：
  imu = ImuSensor()
  tkr.capture_list(imu.raw)        # 加入 ticker 自动采集
  # 在 ticker 回调中：imu.update()  # 运行融合滤波
  yaw = imu.get_yaw()              # 获取偏航角 [-180, 180]

坐标系：
  X = 前, Y = 左, Z = 上
  Yaw = 绕 Z 轴旋转，0° = 初始朝向，正值 = 逆时针 (左转)
"""

import math
from time import ticks_ms, ticks_diff
from seekfree import IMU660RX

# =============================================================================
#                          物理量纲转换
# =============================================================================

# LSM6DSO 参数 (IMU660RX 默认配置)
ACC_LSB_PER_G  = 4096.0    # ±8g 量程 → 4096 LSB/g
GYRO_LSB_PER_DPS = 16.384  # ±2000dps 量程 → 16.384 LSB/(deg/s)
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


def _acc_to_g(raw):
  """加速度计原始 int → g"""
  return raw / ACC_LSB_PER_G


def _gyro_to_radps(raw):
  """陀螺仪原始 int → rad/s"""
  return raw / GYRO_LSB_PER_DPS * DEG_TO_RAD


def _gyro_to_dps(raw):
  """陀螺仪原始 int → deg/s"""
  return raw / GYRO_LSB_PER_DPS


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
  IMU660RX 陀螺仪传感器 + 姿态融合。

  用法：
    imu = ImuSensor()
    tkr.capture_list(imu.raw)   # ticker 自动采集原始数据
    # 在 ticker 回调: imu.update()
    yaw = imu.get_yaw()

  属性：
    raw       — IMU660RX 实例 (传给 capture_list)
    data      — 链接缓冲区 [ax, ay, az, gx, gy, gz]
    yaw       — 当前偏航角 (deg)
    pitch     — 当前俯仰角 (deg)
    roll      — 当前滚转角 (deg)
  """

  def __init__(self, calibrate_samples=100, beta=0.05):
    """
    参数：
      calibrate_samples — 标定采样数 (启动时机器人须静止)
      beta              — Madgwick 增益 (0.01~0.1)
    """
    # 初始化硬件
    self.raw = IMU660RX()
    self.data = self.raw.get()  # 链接缓冲区

    # 滤波器
    self._filter = MadgwickAHRS(beta=beta, sample_freq=100.0)

    # 陀螺仪零偏 (标定后填入，在线持续修正)
    self._bias = [0.0, 0.0, 0.0]

    # 启动标定
    self._calib_samples = calibrate_samples
    self._calib_count = 0
    self._calib_gx = 0.0
    self._calib_gy = 0.0
    self._calib_gz = 0.0
    self._calibrated = False

    # ★ 双缓冲快照：ISR write → _snap[_idx]；主循环 read → _snap[1-_idx]
    self._snap = [0.0] * 8   # 2 slots × 4 quaternion [q0,q1,q2,q3]
    self._snap_idx = 0

    # 在线零偏跟踪 (EMA)
    self._bias_alpha = 0.002   # EMA 平滑因子，~5 秒静止修正 63%
    self._still_count = 0       # 连续静止帧计数
    self._still_needed = 50     # 需要连续 50 帧 (0.5s) 才更新

  # ——————————————————————————————————————————————————————————
  #                      每帧更新
  # ——————————————————————————————————————————————————————————

  def update(self):
    """
    每帧 (ticker 回调) 调用。
    读取最新数据 → 标定/滤波 → 更新姿态。
    应在 capture_list 触发 capture() 之后调用。
    """
    ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = self.data

    # 转换量纲
    ax = _acc_to_g(ax_raw)
    ay = _acc_to_g(ay_raw)
    az = _acc_to_g(az_raw)
    gx = _gyro_to_radps(gx_raw)
    gy = _gyro_to_radps(gy_raw)
    gz = _gyro_to_radps(gz_raw)

    # —— 标定阶段 ——————————————————————————————————————
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
      return  # 标定期间不运行滤波器

    # —— 减去零偏 ——————————————————————————————————————
    gx -= self._bias[0]
    gy -= self._bias[1]
    gz -= self._bias[2]

    # —— 在线零偏跟踪 (静止时 EMA 修正) ——————————————————
    gyro_mag = math.sqrt(gx * gx + gy * gy + gz * gz)
    acc_mag = math.sqrt(ax * ax + ay * ay + az * az)
    is_still = (gyro_mag < 0.0175) and (abs(acc_mag - 1.0) < 0.05)
    # 0.0175 rad/s ≈ 1.0 deg/s

    if is_still:
      self._still_count += 1
      if self._still_count >= self._still_needed:
        # EMA: bias += alpha * residual (residual 即去零偏后的 g)
        a = self._bias_alpha
        self._bias[0] += a * gx
        self._bias[1] += a * gy
        self._bias[2] += a * gz
    else:
      self._still_count = 0

    # —— 运行 Madgwick 融合 ————————————————————————————
    self._filter.update(gx, gy, gz, ax, ay, az)

    # ★ 双缓冲快照（ISR 写，主循环读）
    f = self._filter
    off = self._snap_idx * 4
    self._snap[off]     = f.q0
    self._snap[off + 1] = f.q1
    self._snap[off + 2] = f.q2
    self._snap[off + 3] = f.q3
    self._snap_idx ^= 1

  # ——————————————————————————————————————————————————————————
  #                      姿态获取（读快照）
  # ——————————————————————————————————————————————————————————

  def _read_snap(self):
    return tuple(self._snap[(1 - self._snap_idx) * 4 + i] for i in range(4))

  def get_yaw(self):
    """偏航角, deg, [-180, 180]。未标定完成返回 0。"""
    if not self._calibrated:
      return 0.0
    q0, q1, q2, q3 = self._read_snap()
    yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2),
                     1.0 - 2.0 * (q2 * q2 + q3 * q3))
    return yaw * RAD_TO_DEG

  def get_pitch(self):
    if not self._calibrated:
      return 0.0
    q0, q1, q2, q3 = self._read_snap()
    sin_pitch = 2.0 * (q0 * q1 - q2 * q3)
    if abs(sin_pitch) > 1.0:
      sin_pitch = 1.0 if sin_pitch > 0 else -1.0
    return math.asin(sin_pitch) * RAD_TO_DEG

  def get_roll(self):
    if not self._calibrated:
      return 0.0
    q0, q1, q2, q3 = self._read_snap()
    roll = math.atan2(2.0 * (q0 * q2 + q1 * q3),
                      1.0 - 2.0 * (q1 * q1 + q2 * q2))
    return roll * RAD_TO_DEG

  # ——————————————————————————————————————————————————————————
  #                      原始数据
  # ——————————————————————————————————————————————————————————

  def get_gyro_dps(self):
    """陀螺仪 deg/s (已去零偏)。"""
    raw = self.data
    bx, by, bz = self._bias
    return (
      _gyro_to_dps(raw[3]) - bx * RAD_TO_DEG,
      _gyro_to_dps(raw[4]) - by * RAD_TO_DEG,
      _gyro_to_dps(raw[5]) - bz * RAD_TO_DEG,
    )

  def get_accel_g(self):
    """加速度计 g 值。"""
    raw = self.data
    return (_acc_to_g(raw[0]), _acc_to_g(raw[1]), _acc_to_g(raw[2]))

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
    self._calibrated = False