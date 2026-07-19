# IMU 陀螺仪调参

## 流程

```
cal()           # 1. 标定陀螺 bias
dr(30)          # 2. 静止 30s 漂移测试 → 应 < 0.1°/min
sc(1)           # 3. 手转 360° 刻度标定 → 测 gyro_scale
ref()           # 4. 对准参考线
shake(8)        # 5. 暴力来回转 测 spin_beta 对称性
check()         # 6. 放回参考线看偏差
whirl(3600)     # 7. 电机转 10 圈 测高速漂移
check()         # 8. 放回参考线看偏差
ref() → goto(90) → goto(-90) → goto(0) → check()  # 9. PD 闭环精度
```

## 参数速查

| 参数 | 作用 | 合理范围 |
|------|------|---------|
| `gyro_scale` | 角速度比例 | 0.95-1.15 |
| `beta` | Madgwick 加速度计信任度 | 0.03-0.10 |
| `spin_beta` | 高速转动时 beta 上限 | 0.005-0.02 |
| `spin_dps` | 切换 spin_beta 的 dps 阈值 | 30-60 |
| `bias_alpha` | 静止零偏收敛速率 | 0.001-0.005 |
| `gyro_still` | 判定静止的陀螺阈值 rad/s | 0.01-0.03 |
| `still_need` | 静止确认帧数 | 50-200 |

## 调 gyro_scale

- `sc(1)` 手工转：低速验证，误差应 < 1%
- `whirl(3600)` 电机转 10 圈：**高速验证**，偏差 = 真正的 gyro_scale 误差
- sc 准了但 whirl 漂 → gyro 非线性，高速和低速 scale 不同
- whirl 偏差 > 5°：`gyro_scale *= (1 + net_yaw/3600)` 逐次逼近

## 调 spin_beta

- `shake(8)` 净转角应 < 3°
- 不对称：spin_beta 太小 → 转动中加速度计完全忽略 → 纯陀螺积分漂移
- 对称但 drift 大：spin_beta 太大 → 向心加速度污染 Madgwick
- **spin_beta 持久化 bug**：必须用 `_resting_beta` 持久存储，不能用局部变量 `beta_saved`

## 调 PD (goto)

- `goto(90)` 应 < 1s 收敛到 ±1°
- 振荡：降 kp 或加 kd
- 卡住不动：提高最低占空比（3轮需 6-7%）
- 方向反了：检查 `yaw_actuation_sign`，正占空比看 yaw 增减方向
- 朝反方向转：`err = _chi(cur - target)` 而非 `_chi(target - cur)`

## 调 bias

- `dr(30)` 漂移 > 0.5°/min → 降 bias_alpha
- bias_alpha 太大会追噪声，太小零偏收敛慢
- gyro_still 太大 → 永远不判静止 → bias 不更新
- gyro_still 太小 → 微动也被判静止 → bias 追假零偏
