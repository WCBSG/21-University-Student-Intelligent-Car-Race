# DeepSeek → Grok

---

## [2026-07-14] 比赛模式已实现 — C20 + LED 自动流程

### 新增文件

| 文件 | 内容 |
|------|------|
| `CODE/boot_mode.py` | `read_and_clear()` / `request_reboot(mode)` — `/flash/boot_mode` 文件标志 |
| `CODE/main.py` | 重写：双剖面 MATCH / DEBUG |

### main.py 结构

```
启动 → read_and_clear()
  ├─ BOOT_MODE="MATCH" → 共用 Init → MATCH loop
  └─ BOOT_MODE="DEBUG" → 共用 Init → Display+握手+Menu → DEBUG loop
```

共用 Init（两种模式都跑）：Motors/Arbiter/Config/IMU/CameraRx/TCS/FSM/MatchRunner/Ticker/C20/LED

### MATCH 模式

```
WAIT_CALIB  LED 快闪 200ms  等 imu.is_calibrated
WAIT_CAM    LED 慢闪 500ms  每拍 camera.handshake(retries=1)
READY       LED 常亮        3s 倒计时 → match.start()
RUN         LED 常亮        robot.tick + match.tick
DONE        LED 三快闪      停车

C20 = 急停 (RUN→DONE)
不 import display, 不 init C8/C9/C14/C15
```

### DEBUG 模式

```
原逻辑 + C20 长按 2s → request_reboot("MATCH") → machine.reset()
Display + Menu try/except 不变
```

### 待你做

1. 审一下 MATCH 模式的 LED 闪烁和阶段流转有没有逻辑漏洞
2. P2 (NEXT多件) 和 P3 (HOME回库) 的实现 — 改 MatchRunner 即可，main.py 不变
3. 实测 MATCH 模式省多少 RAM（看 `[MEM]` 或 `[MATCH] phase=X free=Y`）

### 接口不变

MatchRunner API 没改 — start/stop/tick/is_running/phase/scored_count 照旧。你的 P2/P3 只改 `match/runner.py`。
