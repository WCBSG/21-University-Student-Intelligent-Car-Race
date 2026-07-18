# MiniMax → Grok

> 完整分析见 [`archive/MiniMax-to-Grok-2026-07-19-log-analysis.md`](archive/MiniMax-to-Grok-2026-07-19-log-analysis.md)
> 数据源：`CODE/log.txt`（2747 行，**1/3 进球**，耗时 ~294s — 相比 07-18 的 3/3 严重退步）

---

## 待 Grok 确认（按 ROI 排序）

| 优先级 | 改动 | 文件 | 预期效果 |
|--------|------|------|----------|
| 🔴 P0 | PUSH skew 纠偏 timeout 加 4s 上限，失败直接 `abort_repick` | `match_hunt.py:415-449` | 第 1 颗不再丢 15s |
| 🔴 P0 | HUNT 换类加前置条件（每类 ≤ 8s + `vision_lost` 累计） | `match_hunt.py:243-258`、`config.py:pick_timeout_ms` | 减少空转 30s+ |
| 🔴 P0 | BACKOFF 完强制回场心 + 重置 IMU 零位，再进 HUNT | `match_isr.py` + `match.py` | 第 2/3 颗能继续找 |
| 🟠 P1 | dt 钳位阈值 `0.5→0.15`，钳位时保留上一帧 motor 输出 | `main.py:311-314` | PID 不再盲飞 |
| 🟠 P1 | GC 优化：`_build_sensors()` 复用 list/dict | `main.py:176+` | dt clamp 122 → < 10 |
| 🟡 P2 | HUNT_SPIN 加 6 圈硬上限；急转时 IMU 禁 mag | `match_hunt.py`、`imu.py` | yaw 漂移减少 |

### 关键事实（vs 07-18 退步点）

- **进球：1/3**（07-18 是 3/3），主要丢在 PUSH 15s 超时和 BACKOFF 后找不到剩余类
- dt 警告 **122 次**（07-18 是 75 次），最大 **dt=1.00s @ t=78.99s**
- TCS I2C EIO **124 次**（07-18 是 83 次）
- GC `free < 8192` 触发 **455 次**，最低 `free=256 @ t=114.44s`
- PUSH 第 1 次 timeout **15.0s**（skew 纠偏卡死）；第 2 次 7.5s 才得分但 BACKOFF 后丢球
- HUNT 换类 **12 次**（1→2→0→1→2…），换完仍找不到
- yaw 漂移大：err ±150° ~ ±180° 常见，mag=(-1661,…) 跳变说明电机 EMI

### 期望效果

- P0 三项全做 → 3 颗里稳拿 2~3 颗（修单项救不回丢的 2 颗）
- dt clamp 从 122 → < 10 → PID 50Hz 实际生效
- BACKOFF 后能回到场心 → 第 2/3 颗搜索从中心开始