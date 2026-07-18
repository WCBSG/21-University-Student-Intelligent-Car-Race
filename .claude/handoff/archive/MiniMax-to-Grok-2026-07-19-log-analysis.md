# MiniMax → Grok — 2026-07-19 log 分析完整版

> 数据源：`CODE/log.txt`（2747 行，约 294s 赛程）
> 对比 07-18：3/3 进球 → **1/3 进球，严重退步**

---

## 一、关键事件时间线

| 时间 | 事件 | 含义 |
|---|---|---|
| **0.24s ~ 3.48s** | 启动 / IMU 标定 / 相机握手 | 一切正常，READY 后等 C20 |
| **5.62s** | `LEAVE` 出发 | yaw 立刻漂到 +1.9° ~ +2.8° |
| **9.69s** | `LEAVE timeout → HUNT` | 出场 4 秒内没见到目标 |
| **9.89s ~ 32.42s** | **首轮搜索 ~22.6s**：HUNT→ALIGN→超时回 HUNT，多次 reverse-find | 反复 ALIGN 不上 |
| **32.42s** | `ALIGN timeout → HUNT` | 绕行 8s 还凑不齐 yaw+居中 |
| **43.37s** | `ALIGN → PUSH` 第 1 次 PUSH | 推进 15 秒 |
| **58.35s** | `PUSH → CORRECT (skew)` | 推到一半目标横向偏走，启动纠偏子态 |
| **58.45s** | `PUSH timeout 15001ms — NOT scored` | ❌ 第 1 颗球**未得分** |
| **58.53s ~ 153.72s** | **第二轮搜索 ~95s**：反复 HUNT/SPIN→TRACK→lost→reverse-find→ALIGN 失败→HUNT lost long→再 HUNT | 长时间空转 |
| **153.72s** | `ALIGN → PUSH` 第 2 次 PUSH | 推进 7.5 秒 |
| **161.27s** | ✅ `SCORE total=1/3 rem=[2,1,0]` | 中途颜色触发，TCS yellow 进入 BACKOFF |
| **161.34s** | `yellow → BACKOFF (ISR)` | 第 1 颗真正得分 |
| **170.55s / 176.37s** | 又两次 `yellow → BACKOFF (ISR)` | 推第 2 颗时**反复触黄后退**，未成功 |
| **163.08s ~ 294.16s** | **第三轮 ~131s**：HUNT 不停换类 (1→2→0→1→2→0…)，从 `lost=0` 一路累到 `lost=12`，**再也没有进入 PUSH** | 整个后半段几乎空转 |

> **核心数字**：5 分钟赛程 = 1 颗得分 / 至少 12 次 HUNT→ALIGN 循环 / 仅 2 次实际进入 PUSH。

---

## 二、八大主要问题（按严重度排序）

### 🔴 问题 1：主循环严重卡顿，dt 频繁被钳到 0.02s

- 全文 **`MAIN WARN: dt=... clamped → 0.02` 共 122 次**，分布在几乎每一段 HUNT/ALIGN/PUSH 内。
- `main.py:311-314`：
  ```python
  if dt <= 0.0 or dt > 0.5:
    dt = 0.02
  ```
  实际 dt 经常 0.5–1.0s，最大 **dt=1.00s @ 78.99s**。
- 一旦 dt 被钳，`_bearing_pid.update(be, real_dt=0.02)` 算出的微分 / 积分项全错；TCS、IMU、摄像头数据都被陈旧帧带偏。

**根因**：`match.tick()` 内每帧 `_build_sensors()` → `camera.read()` 同步阻塞 + `tkr_tcs` 50Hz tick 抢占 + GC 反复触发（见问题 2）。**dt 钳位等于 PID 在盲飞。**

### 🔴 问题 2：内存长期吃紧，GC 反复拖累主循环

- `_GC_LOW = 8192`，`_GC_CRIT = 3072`。
- 日志里 `gc probe free=768 thr=8192 hit=1` 出现 **455 次**，绝大多数 free 都贴着 2k–6k；最低跌到 `free=256`（t=114.44s）、`free=352`（t=18.07s）。
- 出现多次 **`free_after` 比 `free_before` 还少**（如 t=53.56s：`38128 → 41184`），说明对象已被分配到碎片里，回收收益微弱。
- 反复 `gc.collect()` 单次耗时几十~上百 ms，正好对应那 122 次 dt clamp。

**根因**：相机 UART 解码、`_build_sensors()` 内的临时 list/dict、HUNT 状态机每帧创建 yaw_err/bearing 等浮点对象，加上 HUNT_TRACK/HUNT_SPIN/ALIGN 间切换频繁，碎片化严重。

### 🔴 问题 3：搜索完一轮后失去目标，但没有任何机制能找到剩余类别

- HUNT head 在 `cls=0→1→2→0…` 不断循环（`HUNT head cls=X → end, new head=Y`），但**始终没找到下一个目标**：
  - t=86.40s：cls=1 找不到 → 换 cls=2
  - t=94.56s：cls=2 找不到 → 换 cls=0
  - t=121.44s：cls=0 → 1
  - t=129.62s：cls=1 → 2
  - …（至少 12 次换类）
- 每次换类只是把 `pick_class` 改一下，并没有**先评估场上是否还有该类目标**，更不会**优先朝得分颜色扫一圈**。

**根因**：`match.py:_set_pick_class` + `pick_class_timeout_ms` 设计为「找不到就换类」，但 `pick_timeout_ms=20000` 太宽容，单类可以死磕 20 秒；又因为 yaw 漂移 + dt 钳位，**每类实际有效扫视圈数 ≪ 理论值**。

### 🔴 问题 4：第 1 颗 PUSH 因为「skew」纠偏卡死 → 15s 超时丢分

- t=43.37s 进入 PUSH，t=58.35s 触发 `PUSH → CORRECT (skew)`，t=58.45s `PUSH timeout 15001ms`。
- `push_duty=12.0`（`config.py:60`），但 PUSH 期间实际 `cmd=(+66.0,+0.0,...)` —— **运行值与配置值不一致**，说明还有别处覆写。
- 横向偏走 `correct` 子态启动后，`_write_push_correct` 用 `push_correct_duty` 微调，但**纠偏速度太低 + dt 抖动** → 15s 内纠不回来。
- 第 1 颗没进 BACKOFF → 没检测 yellow → 没得分。

**根因**：PUSH 期间 cx 抖动阈值 (`push_cx_ok`) 太严、或横向 control loop 增益太低，物体一旦滑出就再也拉不回。

### 🟠 问题 5：第 2 颗 PUSH 触发 3 次 yellow → BACKOFF，但都未完成

- t=161.27s 得分后，t=170.55s、t=176.37s 又两次 `yellow → BACKOFF (ISR)`。
- 之后 BACKOFF/RETREAT 后再 HUNT，但**新 HUNT 从 `yaw=-152` 起步，搜了半天进 SPIN/SPIN→TRACK→lost 反反复复**。
- 多次 `cmd=(-50.0,…)` 全速倒车，但 yaw 漂成 ±130°，再来纠正又要 4–8 秒。

**根因**：BACKOFF 后没有 reset 陀螺零位 / 没有「回场心」子阶段，直接进 HUNT 等于从场地边缘开始搜索。

### 🟠 问题 6：yaw 漂移大、IMU 累积误差未消除

- 在 ALGIN/TURN 子态下，err 经常 ±100° ~ ±180°（如 `tgt=+120 err=+150.8`、`tgt=+30 err=-159.2`）。
- 漂移来源猜测：
  1. `reverse_spin` 全靠 `wrap_deg(yaw - rev_start_yaw)` 累加，**没漂移校正**；
  2. 急转 180° 时陀螺零偏在 0.5–1s 大 dt 内被积分放大；
  3. 反复出现 `mrel=+150, +167` 这种巨大相对角，说明 IMU 在 BACKOFF/全速反转时**磁力计被电机电流干扰**（mag=(805,-661,7)、(1126,…)）。

**根因**：陀螺 / 磁力计融合没做"急转时降权"。

### 🟡 问题 7：TCS I2C 抖动

- `TCS I2C EIO x1 (skip frame)` 出现 **124 次**（约每 2.4 秒一次）。
- 单次跳过没问题，但配合 dt 钳位会让"刚读到白线 → 下一秒线没了 → 再下一秒又有"造成边界误判。

**根因**：TCS 34725 与 IMU 共用 I²C 总线，电机会拉低 SDA/SCL；或上拉电阻不够强。

### 🟡 问题 8：开机/READY 内存就只剩 25k

- t=3.48s `loop free=25232`，之后一路掉到 2k–6k。
- 决赛场一上来没有 `gc.collect()` 兜底，相机第一次握手吃掉一大块。

---

## 三、修复建议（按优先级）

### P0 - 必须立即修

| # | 建议 | 文件 |
|---|---|---|
| 1 | **降低 dt 钳位阈值**：把 `dt > 0.5` 改成 `dt > 0.15`，钳到 0.05；并在钳位时**保留上一帧 motor 输出**，避免 PID 跳变。 | `main.py:311-314` |
| 2 | **HUNT 换类前先扫一圈确认场上没目标**：当前是 20s 死磕后盲目换类，应该每类最多 8s 且 `vision_lost` 累计后才换。 | `match_hunt.py:243-258`、`config.py:pick_timeout_ms` |
| 3 | **PUSH skew 纠偏加快**：把 `push_correct_duty` 从 12 提到 25，纠偏 timeout 从无限改成 4s → 失败直接 `abort_repick` 或 `skip_or_home`，别再 15s 死磕。 | `config.py:push_duty/push_correct_duty`、`match_hunt.py:415-449` |
| 4 | **BACKOFF 后强制回场心**：RETREAT 完成后做一次 1m 直行 + 重置 IMU 零位，再进 HUNT；现在的实现是 BACKOFF 完直接 `→ HUNT`，从场地边缘开始找。 | `match_isr.py` + `match.py:_enter_hunt` |

### P1 - 强烈建议

| # | 建议 |
|---|---|
| 5 | **gc 优化**：在 `_build_sensors()` 内复用 list/dict；HUNT_TRACK 里 `target[6]/target[9]` 改成局部变量；相机读到 None 时跳过元组解包。 |
| 6 | **TCS I2C**：拉高 I²C 上拉到 4.7kΩ；或把 TCS 单独挪到 soft-I2C 引脚。 |
| 7 | **IMU 反转期间禁用磁力计**：检测到 `|yaw_rate|>200°/s` 时把 mag 权重降为 0。 |
| 8 | **HUNT_SPIN 阶段限制最大累计转角**：当前没有 spin 圈数上限，看到目标就停 + 触发边界黄线就 `boundary armed`；建议加 6 圈硬上限避免死循环。 |

### P2 - 可选优化

- `push_timeout_ms=3000` 当前配置但日志里跑到 15001ms → `config.py:80` 与运行时不一致，**确认是哪一份配置在生效**（可能 JSON 没刷到 flash）。
- 在 stat 日志里加上 `lost_n`、`confirm_n`、`yaw_rate` 字段，便于现场调试。
- 把"换类"事件从 8s 短超时改成「3s 内没确认 + 累计 lost_n>30」双条件触发。

---

## 四、最小验证清单

跑一遍后看：

1. `MAIN WARN dt clamp` 次数 < 5（当前 122）。
2. `gc probe free < 2048` 次数 = 0（当前 455）。
3. HUNT_TRACK→lost 一次后能在 < 2s 内恢复，不进 reverse-find。
4. PUSH 进入后 8s 内要么得分要么 `abort_repick`，不再有 15s 死磕。
5. 第 2 颗 BACKOFF 后能回到场心，再 HUNT。

预计修完 P0 三项后，**3 颗里稳定拿 2~3 颗**是合理的；只修单项无法挽回丢掉的 2 颗球。

---

## 五、与 07-18 对比

| 指标 | 07-18 | 07-19 | 变化 |
|------|-------|-------|------|
| 进球数 | 3/3 | 1/3 | 🔴 -2 |
| 赛程耗时 | 215s | 294s | 🟠 +79s |
| dt clamp 次数 | 75 | 122 | 🟠 +47 |
| TCS EIO 次数 | 83 | 124 | 🟠 +41 |
| PUSH timeout 次数 | 2 (15.7+15.5s) | 1 (15.0s) | -1 但仍是主因 |
| HUNT 换类次数 | — | ≥12 | 新增现象 |
| yaw 最大 err | — | ±180° | 新增现象 |

07-19 的 3 颗球有 2 颗丢在：
- 第 1 颗：PUSH skew 15s 超时
- 第 3 颗：BACKOFF 后找不到（换类 12 次仍未找到）

修这两条即可直接挽回。