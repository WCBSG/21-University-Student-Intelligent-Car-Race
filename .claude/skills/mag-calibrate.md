# 磁力计标定与融合

## 流程

```
mag()           # 1. 看 raw XYZ 范围，确认数据稳定
mag_cal()       # 2. 慢转 360° 收集 min/max → 得 ox/oy
set('mag_ox', ox)  # 3. 设硬铁偏移
set('mag_oy', oy)
set('mag_on', True) # 4. 开磁力计
cal()           # 5. 重新标定 IMU
ref()           # 6. 对准参考线（同时重置 mag_ref）
dr(30)          # 7. 看磁修正是否稳在 0 附近
```

## 硬铁标定

- `mag_cal()` 旋转时 X/Y 画圆，中心 = 硬铁偏移
- 偏移 > 50 正常（电机永磁体靠近 IMU）
- 偏移过大（>2000）→ 检查电机/磁铁是否太近
- **磁力计只在静止且电机停转时修正**，转动中不受电机磁场干扰

## 磁融合参数

| 参数 | 作用 | 建议值 |
|------|------|--------|
| `mag_alpha` | 磁修正收敛速率 | 0.002-0.005 |
| `mag_dead` | 死区(°) — 小于此不修正 | 1.0-2.2 |
| `mag_pull` | 单帧最大修正(°) | 5-10 |
| `mag_still_n` | 静止确认帧数 | 60-100 |
| `mag_lpf` | 磁 heading 低通 | 0.005-0.02 |

## 调试关键

### mag_ref 设置时机
- 首次激活时 `mag_ref = mag_heading - prev_yaw`
- **若此时 mad 已漂移 → mag_ref 永久偏移 → 磁修正拉偏**
- 解决：`ref()` 重置 `_mag_ref = None`，让参考线位置成为磁零位
- 教训：绝对参考系初始化前，mad 必须已稳定

### fused_offset 收敛
- `yaw = mad + fused_offset`，偏移补偿 mad 漂移
- goto 后 mad 可能漂 4-5° → fused_offset 自动补偿 → 30-60s 收敛
- dr(30) 静止时磁修正把 yaw 锁在 -0.6°（vs mad 漂了 1.3°）

### 常见问题
- `off=0.00` 不变 → 检查 `_mag_ref` 是否 None（AttributeError 被 `except:pass` 吞）
- mag heading 不变化 → 检查 `mag_ox/oy` 是否设对
- 磁修正把 yaw 拉偏 → mag_ref 在错误位置初始化 → ref() 重置
