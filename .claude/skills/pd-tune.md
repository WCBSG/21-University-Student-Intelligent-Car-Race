# PD 控制器调参

## goto PD 模板

```python
def goto(target, max_duty=40, kp=1.0, kd=0.08, tol=1.5):
    prev_yaw = _yaw()
    while True:
        cur = _yaw()
        err = _chi(cur - target)          # + = 已过目标
        dps_est = _chi(cur - prev_yaw) / 0.02  # signed 转速
        prev_yaw = cur

        if abs(err) <= tol:
            settle += 1
            if settle >= 10: break
            out = -kd * dps_est          # 纯 D 制动
        else:
            settle = 0
            out = kp * err + kd * dps_est # PD
            if out > max_duty: out = max_duty
            elif out < -max_duty: out = -max_duty
```

## 参数经验

| 参数 | 推荐值 | 作用 |
|------|--------|------|
| kp | 1.0 | 比例增益，err=90°→out≈90%→钳位到 max_duty |
| kd | kp/12.5 ≈ 0.08 | 阻尼，30°外+400dps→out≈30-32≈-2→提前刹车 |
| max_duty | 40 | 限制最大占空比，防全速过冲 |
| tol | 1.5° | 容忍区，太小→永远收敛不了；太大→精度差 |

## 调试步骤

### 1. 方向验证
- 正 err → 正 out（通过 `err = _chi(cur - target)` 确保）
- 正 out → CCW（通过实验确认：`_arb.write(OWNER, [30,30,30])` 看 yaw 变化）
- 若方向反：在 `_hdg_pid.update()` 外层用 `yaw_actuation_sign` 翻转

### 2. 振荡排查
- **180° 对面振荡** → err 方向判断错了 → 检查 `_chi` 的参数顺序
- **目标附近来回摆** → kp 太大 / kd 太小 / max_duty 太高
- **接近目标就停** → 最低占空比不够 → 检查 setSpeed MIN_DUTY

### 3. 收敛速度
- 120° → 1°：目标 < 1s
- 若 > 3s：kp 太小或 max_duty 太低
- 若 < 0.5s 但过冲大：降低 max_duty 或加 kd

## 从开环迁移到 PD

识别模式：
```python
# 开环：手写分档减速
if abs(err) < 40: s = s * 0.45
elif abs(err) < 80: s = s * 0.7

# → PD：自动减速
s = sign * hdg_pid.update(err, dt, rate)
```

迁移清单：
- `_spin_toward()` ✓
- `step_backoff` SPIN ✓
- `_align_lost_soft` 搜索 ✓
- `_write_move_locked` 航向保持 ✓
- `_tick_align` 4 处航向 PID ✓
- `_tick_hunt_track` bearing PID ✓
- `_write_push_correct` bearing PID ✓

## HeadingPID 升级

```python
class HeadingPID:
    def update(self, error, dt, rate=0.0):
        kp, mx, db, kd = self._params()
        if abs(error) < db: return 0.0
        out = kp * error - kd * rate   # D 抑制当前转速
        return clamp(out, -mx, mx)
```

- `rate` = signed yaw 变化率 (°/s)，正=CCW
- D 项 = `-kd * rate`：转速越快，出力越小 → 自动刹车
- 向后兼容：不传 rate → kd=0 → 纯 P

## 常见错误

- D 项符号反：`+kd*rate` 加速而非制动 → 发散振荡
- dt 为 0：导致 rate = ∞ → 检查 `_control_dt()` 的 clamp 逻辑
- 多个调用路径共享 `_ctrl_ms`：确保互斥（不同 phase 不会同时调用）
