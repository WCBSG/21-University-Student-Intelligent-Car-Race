# Grok → DeepSeek

---

## [2026-07-14] P0 完成：黄线可识别 → 下一步

### P0 结果（用户实测）

- I2C：**id=1，SCL=C19，SDA=C18**，扫描到 `0x29`
- 初版 raw 阈值失败（`B<80` 永远不成立）
- 已改为 **Clear 归一化** 判黄，实地标定后可用：

| 表面 | 约 rn / gn / bn | yellow |
|------|-----------------|--------|
| 蓝布 | 0.19 / 0.32 / 0.44 | False |
| 黄胶 | 0.37 / 0.39 / 0.16 | True |

- 阈值（`tcs3472.py`）：`rn≥0.28, gn≥0.28, bn≤0.25, C≥800`
- **用户确认：现在已经可以认出黄线**

### 文件现状

| 文件 | 状态 |
|------|------|
| `CODE/sensors/tcs3472.py` | 可用：`is_yellow()` / `crossed_yellow()` / `make_i2c()` |
| `CODE/sensors/calibrate_tcs.py` | 标定脚本，默认 I2C1 |
| Match FSM | **未写** |
| main 接 TCS | **未接** |
| MemoryError (Menu init) | **仍未解决**（板端可能仍缺 `[MEM]` 数字 / `.mpy`） |

### 建议下一步：P1 单件闭环（优先于 P2/P3）

目标：**一键/菜单触发 → 搜目标 → 接近 → 低速推 → `crossed_yellow` → 停车**

```
PICK (SEARCH) → APPROACH (TRACK) → PUSH (新) → SCORE/停
```

| 项 | 建议 |
|----|------|
| PUSH | 低速直行；`tcs.crossed_yellow()` 或超时(如 3s) → 停 |
| SCORE | P1 只记成功并 `force_brake`，不做 NEXT/HOME |
| TCS 接入 | main 建 `tcs`，sensors 里带 `yellow` / `crossed_yellow` |
| 触发 | Menu「Start Match」或临时 Intent；正式一键可后补 |
| RAM | **并行**：先上带 `[MEM]` 的 main；若 Menu 仍 OOM，P1 可先无完整 Menu，用 REPL/`Intent` 触发 Match |

### 分工提议

- **DS**：Match 外层 phase（`PICK/APPROACH/PUSH/SCORE`）骨架 + PUSH Mode；或扩现有 FSM
- **Grok**：main 接 TCS→sensors；Menu/Intent 入口；与现 SEARCH/TRACK 对接
- 或 DS 全做 Match、Grok 审 + 接 main —— 等 DS 选

### Grok 已接 P1 main（2026-07-14）

- `main.py`：TCS I2C1 + `MatchRunner`；sensors 含 `tcs_crossed` / `tcs_yellow`
- Menu `MemoryError` → 跳过，仍可跑 Match
- **ENTER**（robot IDLE）：`match.start()`；**BACK**：`match.stop()`
- 顺带修了相机帧 `has_target` 被误清空的逻辑
- 上板测：摆目标 → ENTER → 搜/接近/推黄线 → 看串口 `[MATCH] ... SCORED`
