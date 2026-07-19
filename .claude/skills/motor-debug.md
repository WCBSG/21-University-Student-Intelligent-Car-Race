# 电机调试与最低占空比

## 电机极性验证

```python
# 验证：正占空比看 yaw 变化
ref()
_arb.acquire("TEST")
_arb.write("TEST", [30, 30, 30])  # 正占空比
sleep_ms(500)
_arb.force_brake()
check()  # yaw 减小 → 正=CW；yaw 增大 → 正=CCW
```

## 最低占空比

- 单个轮子：7% 才转
- 3 个轮子同时：6% 可缓慢转
- **放在 `setSpeed()` 全局处理**：

```python
MIN_DUTY = 7

def setSpeed(self, duties):
    for d in duties:
        if 0 < d < MIN_DUTY: d = MIN_DUTY
        elif -MIN_DUTY < d < 0: d = -MIN_DUTY
        # d=0 不提升（刹车/停止）
```

- `hold_brake` / `force_brake` 直接写 PWM → 不受最低占空比影响
- 调用层（如 goto PD）不需要单独处理最低占空比

## 电机控制架构

```
MatchRunner._write_*  →  MotorArbiter.write  →  MotionControl.setSpeed  →  PWM
      调用层                  仲裁层                   物理层（最低占空比在此）
```

## PD 输出与最低占空比协作

- goto PD 输出 3% → `setSpeed` 提升到 7% → 能转动
- goto PD 输出 0 → `setSpeed` 不动 → 刹车/停止正确
- 在 PD 层也保留 boost（`if 0 < out < 7: out = 7`）保障 PD 方向判断正确

## 运动学加速

- `MotionControl.move(speed, 0)` 每帧调 sin/cos → **用 `move_forward(speed)` 替代**
- `MotionControl.move(lateral, ±90)` → **用 `move_side(speed)` 替代**
- 预计算常量：`_FWD_K = 1/√3 ≈ 0.5774`, `_SIDE_K = 1/3 ≈ 0.3333`

```python
@staticmethod
def move_forward(speed):    # 免 trig
    s = speed * _FWD_K
    return [s, s, 0.0]

@staticmethod
def move_side(speed):      # 免 trig
    s = speed * _SIDE_K
    return [-s, -s, 2.0 * s]
```

## 已验证的运动学

**极性**：`[d,d,d]` 正占空比 → CW（yaw↓）。`yaw_actuation_sign = -1`。

**轮序**：轮1 物理接线方向与运动学模型相反，需取反。`move()` 通用公式本身正确，但 `move_forward`/`move_side` 是手写 bug：

```python
# move() — 正确的通用运动学（已验证）
def move(speed, angle):
    r = math.radians(-angle)
    c = math.cos(r) / math.sqrt(3)
    s = math.sin(r) / 3
    return [speed*(s+c), speed*(s-c), speed*(-2*s)]

# move_forward(s)  = move(s, 0)    = [s, -s, 0]
# move_side(s)     = move(s, -90)  = [s, s, -2s]  (右移)
# move(s, 90)                     = [-s, -s, 2s]  (左移)
```

`move_forward` 原来手写 `[s, s, 0]` → 车横着跑。修复为 `[s, -s, 0]`。
`move_side` 原来手写 `[-s, -s, 2s]` → 左移。修复为 `[s, s, -2s]` = 右移。

## 常见问题

- 低速抖动不转 → 低于最低占空比 → 全局提升到 7%
- PD 收敛后振荡 → 死区 `abs(out) < 5 → 0` 太宽，与最低占空比冲突
- settle 阶段过冲 → 容忍区内不用 0，用 `-kd * dps_est` 制动
- 车横着跑 → 运动学公式错误 → 用 `move(speed, angle)` 验证
