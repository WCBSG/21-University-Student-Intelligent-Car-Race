"""
config.py — 比赛固件参数（模块级变量，import config as cfg）

所有配置直接写在此文件，构建脚本 build_flash.py 会去除注释和 docstring。
发车前只需改此处数值，不需碰 JSON。
"""
# ═══════════════════════════════════════════════════════════════
#  类别常量（与 OpenART 检测类别一致，不可改）
# ═══════════════════════════════════════════════════════════════
#   0 = 沙袋(场左)  1 = 网球(场上)  2 = 熊(场右)  3 = XB(库区信标)  4 = brick(红砖干扰物)
CLS_LEFT  = 0                    # 沙袋
CLS_UP    = 1                    # 网球
CLS_RIGHT = 2                    # 熊
CLS_XB    = 3                    # XB 信标（HOME LEG2 二次定位）
CLS_BRICK = 4                    # 红砖（干扰物，不可推，须绕开）

# ═══════════════════════════════════════════════════════════════
#  比赛模式
# ═══════════════════════════════════════════════════════════════
match_mode = "final"             # "pre"=预赛（直推出线）, "final"=决赛（绕行对位推箱）
match_target_count = 5           # 要推的物体总数（得分 N 次后回库）
start_layout = 2                 # 发车位置: 1=底边中  2=左下角  3=右下角
match_order = [CLS_UP, CLS_RIGHT, CLS_LEFT]  # HUNT 搜索优先级 [网球,熊,沙袋]
strict_target = False            # True=只推 single_target_class 指定类别
single_target_class = 0          # strict_target=True 时生效: 0/1/2

# ═══════════════════════════════════════════════════════════════
#  占空比（PWM 百分比，范围 -100 ~ 100）
# ═══════════════════════════════════════════════════════════════
drive_duty = 60.0                # LEAVE / HOME / BACKOFF 共用直行速度
push_duty = 75.0                 # PUSH 推物直行速度
leave_shift_duty = 50.0          # LEAVE SHIFT 横向平移速度
push_correct_duty = 35.0         # PUSH 过偏时慢速纠偏速度

# ═══════════════════════════════════════════════════════════════
#  超时（毫秒）
# ═══════════════════════════════════════════════════════════════
drive_timeout_ms = 4000          # LEAVE EXIT 直行最大时间
leave_shift_ms = 4000            # LEAVE SHIFT 横向平移最大时间
push_timeout_ms = 30000          # PUSH 推物最大时间
recover_backoff_ms = 2500        # BACKOFF 后退最大时间
home_timeout_ms = 30000          # HOME 回库最大时间
orbit_timeout_ms = 8000          # ALIGN 单阶段最大时间
approach_cluster_timeout_ms = 15000  # ALIGN 绕物总硬超时
pick_timeout_ms = 20000          # HUNT 搜索超时
center_fwd_ms = 4000             # HUNT FWD 前进寻物超时

# ═══════════════════════════════════════════════════════════════
#  PUSH 推箱监护
# ═══════════════════════════════════════════════════════════════
push_watch_frames = 4            # 连续 N 帧丢失目标 → 重新找物
push_entry_grace_ms = 400        # ALIGN→PUSH 后视觉遮挡宽限期
push_cx_left_min = 46.0          # 目标 cx 下限（%，扩大死区减少震颤）
push_cx_right_max = 54.0         # 目标 cx 上限（%，扩大死区减少震颤）
push_lost_blind_ms = 300         # 进入盲区后仍可继续推的时限
push_correct_lateral_kp = 0.45   # PUSH 纠偏横移 P 增益
push_correct_lateral_kd = 0.12   # PUSH 纠偏横移 D 阻尼
push_correct_lateral_max = 35.0  # PUSH 纠偏横移上限
push_evade_forward_duty = 25.0   # brick 绕障时前进速度
push_evade_lateral_duty = 50.0   # brick 绕障时侧移速度
push_evade_min_ms = 350          # 绕障至少持续时间
push_evade_clear_frames = 3      # 连续无障碍帧数后退出绕障
push_slow_zone_y2 = 85.0         # 目标接近黄线时开始降速
push_slow_duty = 45.0            # PUSH 末段速度，减少黄线过冲
path_brick_cx_tol = 30.0         # brick 与目标路径中心的横向判定范围
path_brick_min_y2 = 50.0         # 无目标时正前方 brick 的避障距离阈值
hunt_evade_ms = 550              # HUNT 路径遇障时斜移持续时间
hunt_evade_forward_duty = 30.0   # HUNT 避障前进分量
hunt_evade_lateral_duty = 45.0   # HUNT 避障侧移分量

# ═══════════════════════════════════════════════════════════════
#  BACKOFF 后退
# ═══════════════════════════════════════════════════════════════
backoff_retreat_min_ms = 800     # 后退最少保持时间
backoff_spin_deg = 180.0         # 闭环 PD 自旋目标角度（°），配合 yaw_tol 消抖
backoff_spin_tol_deg = 3.0       # BACKOFF 自旋完成容差
backoff_spin_timeout_ms = 3000   # 自旋硬超时
backoff_spin_confirm_frames = 5  # 航向达标连续确认帧数
backoff_spin_max_duty = 30.0     # BACKOFF 自旋限幅，防 180°附近反转震荡
turn_latch_release_deg = 80.0    # 180°转向保持初始方向直到误差低于此值

# ═══════════════════════════════════════════════════════════════
#  HOME 回库
# ═══════════════════════════════════════════════════════════════
home_backoff_ms = 1500           # HOME 离线后退时间
home_cross_ms = 1000             # HOME 过线后继续前进时间
home_xb_contact_y2 = 75.0        # LEG2 阶段 XB 信标 y2≥此值 → 接近确认
home_xb_lateral_kp = 0.6         # XB 信标侧移修正 P 增益（同 push_correct）
home_xb_lateral_kd = 0.5         # XB 信标侧移修正 D 阻尼
home_mag_settle_ms = 1000        # 回库前最少停车时间
home_mag_settle_max_ms = 1800    # 地磁未就绪时最多等待时间
home_turn_kp = 0.65              # HOME 转向比例增益
home_turn_kd = 0.22              # HOME 转向角速度阻尼
home_turn_max_duty = 25.0        # HOME 转向限幅
home_turn_rate_tol = 15.0        # HOME 转向完成角速度阈值
home_turn_confirm_frames = 8     # HOME 转向连续稳定帧数
home_turn_timeout_ms = 7000      # 单次 HOME 转向超时
home_leg2_min_ms = 1500          # LEG2 最短行驶时间，忽略刚离开的黄线
home_xb_wait_ms = 1200           # 压线后等待 XB 确认时间
home_require_xb = True           # 无 XB 不把任意黄线当作库门

# ═══════════════════════════════════════════════════════════════
#  航向
# ═══════════════════════════════════════════════════════════════
align_tol_deg = 10.0             # 航向对齐容差（°）
yaw_actuation_sign = -1.0        # 三轮同正值时的旋转方向: -1=顺时针(yaw↓)

# ═══════════════════════════════════════════════════════════════
#  ALIGN 绕行对位
# ═══════════════════════════════════════════════════════════════
orbit_speed = 25.0               # 航向对齐后侧移速度
orbit_front_spin = 16.0          # 绕轴自旋分量（过小会卡在静摩擦）
orbit_front_slip = 140.0         # 绕轴侧移分量（相对 spin 越大轴距越大）
orbit_front_flip = False         # True=轴在车后时翻转侧移符号
orbit_radial_kp = 1.2            # 径向保距 P 增益
orbit_radial_max = 12.0          # 径向修正上限（%）
orbit_yaw_tol_deg = 8.0          # ALIGN 航向对齐容差
orbit_center_tol_pct = 8.0       # 侧移居中容差（%）
orbit_confirm_frames = 3         # 居中确认帧数 → CLOSE
orbit_lost_frames = 3            # 丢目标等待帧数 → 重搜
orbit_backoff_duty = 45.0        # 绕行前快速后退速度
orbit_backoff_min_ms = 250       # 绕行前至少后退时间
orbit_min_slip = 28.0            # 绕轴最小侧移，必须能克服静摩擦
orbit_min_spin = 8.0             # 绕轴最小自旋，避免卡死
orbit_translate_yaw_deg = 20.0   # 小误差才切平移；过大导致绕轴区被压死
orbit_brake_start_deg = 55.0     # 接近平移区才开始减速
orbit_brake_min_scale = 0.55     # 减速区最低输出比例（防抖但不能低于可动）

# ═══════════════════════════════════════════════════════════════
#  磁力计（硬铁校准值，IMU 坐标）
# ═══════════════════════════════════════════════════════════════
mag_enabled = True               # 启用磁力计融合
mag_ox = 1733.0                  # 硬铁 X 偏移
mag_oy = -437.5                  # 硬铁 Y 偏移
mag_oz = -930.0                  # 硬铁 Z 偏移

# ═══════════════════════════════════════════════════════════════
#  IMU（陀螺 + 加计 + 磁融合）
# ═══════════════════════════════════════════════════════════════
imu_calibrate_samples = 400      # 陀螺初始标定采样帧数（2s，降低零偏误差）
imu_beta = 0.05                  # Madgwick 滤波增益
imu_gyro_still = 0.0175          # 陀螺静止阈值（rad/s）
imu_acc_still = 0.05             # 加计静止阈值（g 偏差）
imu_bias_alpha = 0.002           # 陀螺零偏校正速率
imu_mag_alpha = 0.005            # 磁融合速率
imu_mag_dead = 2.2               # 磁融合死区（°）
imu_mag_pull_max = 6.7           # 磁融合单次修正上限（°）
imu_mag_still_need = 100         # 磁融合静判帧数（200Hz）
imu_still_needed = 100           # 零偏校正静判帧数（200Hz）
imu_mag_lpf_alpha = 0.01         # 磁参考 heading 低通 α
imu_gyro_scale = 1.0             # 陀螺刻度（>1 放大角速度）
imu_spin_beta = 0.01             # 高速转动 Madgwick beta 上限
imu_spin_dps = 40.0              # 转动角速度阈值（°/s）

# ═══════════════════════════════════════════════════════════════
#  航向 PID（前向/自旋时锁 yaw）
# ═══════════════════════════════════════════════════════════════
heading_kp = 0.55                # 比例增益，降低锁航向过冲
heading_kd = 0.08                # 配合滤波角速度，避免 D 项反向抽动
heading_max = 18.0               # 航向修正限幅
heading_db = 2.5                 # 扩大静止死区，消除小角度来回修正
heading_slew_duty_s = 100.0      # 航向输出每秒最大变化量
yaw_rate_lpf_tau = 0.08          # 控制用角速度低通时间常数（秒）

# ═══════════════════════════════════════════════════════════════
#  跟踪 bearing PID（视觉追踪时修正朝向）
# ═══════════════════════════════════════════════════════════════
tracking_kp = 1.2                # 比例增益
tracking_kd = 0.05               # 微分阻尼
tracking_max = 60.0              # 输出上限（%）
tracking_db = 0.02               # 死区（归一化）

# ═══════════════════════════════════════════════════════════════
#  视觉跟踪参数
# ═══════════════════════════════════════════════════════════════
tracking_approach_speed = 55.0   # HUNT TRACK 前向接近速度（%）
tracking_final_approach_speed = 35.0  # ALIGN CLOSE 最终接近速度（%）
tracking_search_speed = 80.0     # 降低搜索角速度，避免跨目标反复回摆
tracking_spin_rate_kp = 0.15     # 搜索角速度闭环增益
tracking_spin_max_duty = 14.0    # 搜索旋转输出限幅
tracking_spin_slew_duty_s = 80.0 # 搜索输出斜坡，禁止瞬间正反切换
tracking_target_class = 7        # 目标类别: 0/1/2 或 7=全部
tracking_min_confidence = 18     # 最低置信度（0-31，越大越严格）
tracking_suspect_min_confidence = 12  # 疑似目标阈值，触发停车观察
tracking_confirm_frames = 1      # 锁定目标需连续帧数
tracking_lost_frames = 3         # TRACK 丢目标容忍帧数
tracking_observe_ms = 1000       # 疑似目标停车观察/磁力计修正时间
tracking_observe_cooldown_ms = 1200  # 观察失败后再次触发的冷却时间
push_heading_realign_deg = 45.0  # PUSH 航向偏差过大时停止并重新 ALIGN
push_heading_realign_frames = 2  # 连续超限帧数
tracking_stop_bottom_pct = 93.0  # 预赛：目标 y2≥此值 → 停止（接触判定）
tracking_stage_bottom_pct = 75.0 # 决赛：目标 y2≥此值 → HUNT→ALIGN
tracking_contact_bottom_pct = 94.0  # 决赛：目标 y2≥此值 → ALIGN→PUSH
tracking_bearing_actuation_sign = 1.0  # bearing→旋转方向的符号
tracking_cam_timeout_ms = 5000   # 相机无帧超时（ms）
tracking_stale_ms = 300          # 无新检测帧后清除旧目标
camera_poll_max_frames = 2       # 主循环每拍最多解析的 UART 帧数
camera_poll_budget_ms = 4        # 主循环解析相机的时间预算

# ═══════════════════════════════════════════════════════════════
#  TCS3472 黄线检测
# ═══════════════════════════════════════════════════════════════
tcs_confirm_n = 2                # 黄线消抖确认帧数
tcs_push_confirm_n = 1           # PUSH 阶段快速场锁确认帧数
tcs_r_min = 0.28                 # 归一化 R 下限（r = R/C）
tcs_g_min = 0.28                 # 归一化 G 下限
tcs_b_max = 0.25                 # 归一化 B 上限
tcs_c_min = 800                  # Clear 通道下限（防暗处误判）
tcs_poll_ms = 20                 # 主循环 TCS 采样周期（50Hz）

# ═══════════════════════════════════════════════════════════════
#  HUNT 搜索
# ═══════════════════════════════════════════════════════════════
pick_class_frames = 20           # 队首目标 N 帧未出现 → 换类

# ═══════════════════════════════════════════════════════════════
#  调试
# ═══════════════════════════════════════════════════════════════
debug_output = True              # True=保留 info/log 日志; False=构建时剥除
