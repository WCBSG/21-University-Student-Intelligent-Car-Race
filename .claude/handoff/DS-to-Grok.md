# DeepSeek → Grok

> 完整分析见 [`archive/ds-to-grok-2026-07-19-log-analysis.md`](archive/ds-to-grok-2026-07-19-log-analysis.md)
> 数据源：`CODE/log.txt`（2747 行，**仅 1/3 进球**，耗时 ~294s 仍未完赛）

---

## 待 Grok 确认（按 ROI 排序）

| 优先级 | 改动 | 文件 | 预期效果 |
|--------|------|------|----------|
| 🔴 P0 | 排查内存泄漏 — gc free 6 次 <1000B（最低 256B） | `main.py` / `match.py` | GC 阻塞消失 → dt 正常 → 帧率回升 |
| 🔴 P0 | TCS 加外部上拉电阻 (4.7kΩ→3.3V) 到 C19/C18 | 硬件 | I2C EIO ~40 次 → 大幅下降 |
| 🔴 P0 | ALIGN 旋转降速：`orbit_front_spin` 25~30 | `config.py` | 目标不出视野 → ALIGN lost 减半 |
| 🟠 P1 | `lost_frames` 调到 6~8，`push_timeout_ms` 降到 8000 | `config.py` | 减少误丢 + PUSH 不再耗 15s |
| 🟠 P1 | PUSH skew 纠偏加 timeout 上限 4s | `match_hunt.py` | 避免纠偏卡死 |
| 🟡 P2 | HUNT 旋转期间 IMU 禁 mag pull | `match_hunt.py` / `imu.py` | yaw 稳定 |

### 关键事实

- **仅得 1 分**，目标 3 分
- dt 警告 **~70 次**，最差 dt=1.06s（PID 实际 ~1Hz）
- TCS I2C EIO **~40 次**
- gc free 极小值：256 / 240 / 384 / 784 / 928 / 1056
- ALIGN lost → HUNT **~20 次**，最大时间浪费
- PUSH 唯一得分靠 yellow ISR，不是正常推动到位
- IMU yaw 异常跳变：+4.7°→-130°，+53°→-166°，+66°→-123°
- CAM filtered + HUNT cls 切换频繁，在 0/1/2 类间反复跳

### 期望效果
- 内存稳定 → dt < 0.05s → PID 恢复正常
- ALIGN 不再死循环 → 总耗时 ↓60s
- TCS 稳定 → 黄线检测可靠 → 得分不只靠超时兜底
