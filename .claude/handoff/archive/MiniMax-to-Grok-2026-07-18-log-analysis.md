# 完整日志诊断报告 — 2026-07-18

> 数据源：`CODE/log.txt`（1564 行）
> 结果：3/3 进球 + HOME 完成，但总耗时 ~215s（理想应在 90s 内）

---

## 总体问题表

| # | 问题 | 频次 | 严重度 |
|---|------|------|--------|
| 1 | 主循环 dt 警告（卡顿） | **75 次** | 🔴 致命 |
| 2 | TCS I2C EIO 通信失败 | **83 次** | 🔴 致命 |
| 3 | PUSH 推动 15.7s 超时 | 2 次 | 🟠 严重 |
| 4 | HUNT lost / 反转旋转 | **25 次** | 🟠 严重 |
| 5 | ALIGN lost / reverse find | 15 次 | 🟡 中等 |
| 6 | CAM filtered 频繁过滤 | 18 次 | 🟡 中等 |
| 7 | IMU yaw 漂移（mrel 大跳） | 多处 | 🟡 中等 |

---

## 🔴 问题 1：主循环 dt 卡顿（75 次警告）

**症状**
```
[MAIN] WARN: dt=0.51s clamped → 0.02
[MAIN] WARN: dt=0.64s clamped → 0.02
[MAIN] WARN: dt=0.93s clamped → 0.02    ← 最严重
```
dt 实际值常达 **0.5~0.9 秒**！被强制 clamp 成 0.02，PID 积分爆掉被压制。

**根因排查**
1. `main.py:272` 主循环每次 `sleep_ms(5)` 结尾 → 正常应 < 30ms
2. `match.py:206` `_write_move_locked` 内的 dt 计算 → 单次不会卡
3. `tcs3472.py:166-169` 一旦发 `readfrom_mem` 失败就抛 OSError → 但被 `try/except` 吞掉
4. `tcs.isr_field_lock` 在 ISR 中调，但 bit-bang I2C **占用 CPU 时间长**

**特别可疑**
- `tcs3472.py:103` 用 GPIO bit-bang I2C（100kHz），4 个寄存器 (16bit×4 + 2 reg addrs + START/STOP) ≈ **30~60 次 GPIO 翻转 + 30~60 次 `sleep_us(5)`** ≈ **300~600μs 每帧**
- 100Hz ISR 中跑这个，理论上只占 3~6% CPU，但若 bit-bang 期间被打断，可能拖更久

**影响**：PID 控制频率实际只有 ~2Hz 而不是预期的 50Hz，导致：
- 推动时响应迟钝
- 转向超调
- TCS 漏帧 → 黄线检测滞后 → BACKOFF 触发延后

**修复建议**
- 检查是否在主循环某处有阻塞调用（gc.collect? print? array分配?）
- 考虑把 `info()` 里的 f-string 改成普通字符串拼接
- 缩短 `_build_sensors()` 中重复的 `ticks_diff` 计算

---

## 🔴 问题 2：TCS I2C EIO 高频失败（83 次）

**症状**
```
[TCS] I2C EIO x1 (skip frame)    ← 几乎每 2-3 行出现一次
```
83 次记录只是**每 2 秒报一次**，实际错误率远高于此（看 `tcs3472.py:256` 的去抖逻辑）。

**根因**
1. **`tcs3472.py:103` `make_i2c` 用 bit-bang 软 I2C**（C19/C18），抗干扰能力差
2. **电机 PWM 13kHz** 在 `motion.py:31-35` 三路同时切换 → 电源纹波 + EMI
3. 线缆没屏蔽 / 离电机太近
4. 上拉电阻：bit-bang I2C 用的是 `OPEN_DRAIN` 但**没显式上拉** → C19/C18 内部弱上拉不够

**特别看日志验证干扰**
- 行 254：`mag=(-1661,-1616,2265)` — 磁场瞬间 ±1600+，电机电流的典型特征
- 行 117：`mag=(-641,-261,-105)` → 行 122 `mag=(-641,-346,-66)` → 行 148 `mag=(234,851,-161)` — yaw 计算输入不稳

**修复建议**
- 加外部上拉电阻（4.7kΩ 到 3.3V）
- 或改用硬件 I2C（但要避开摄像头/IMU 用的引脚）
- TCS 接线远离电机 / 加屏蔽
- 调试时用示波器看 SCL/SDA 波形

---

## 🟠 问题 3：PUSH 推动 15.7s 超时（2 次失败）

**症状**
```
[MATCH] PUSH timeout 15722ms — NOT scored
[MATCH] PUSH timeout → skip cls=1
```
两次推动都失败（最终是 BACKOFF 时 yellow ISR 触发的得分，不是推动后看到的）。

**根因分析（看推动过程的 log）**
- 行 612：`yaw=-70.5 tgt=-67.5 err=+2.9 cx=44.9 y2=99.6 cmd=(+66.0,+0.0,-14.8)`
  → 差速 -14.8，左轮 23.3 / 右轮 -52.9（一侧反转）！
- 行 616：`TCS R=355 G=584 B=818 C=1833` — 颜色正常但 cx 没了 → 失去目标
- 行 661：`TCS R=229 G=347 B=494 C=1107` — **光线骤降到 C=1107**（正常 ~1900）
- 行 703：`TCS R=210 G=324 B=460 C=1031` — 仍很暗
- 行 706：`yaw=-30.6 tgt=-67.5 err=-19.6 cmd=(+66.0,+0.0,+40.7)` — **朝错误方向差速！**

**问题本质**
1. **推动时车身姿态偏 → 一侧电机反转 → 反复空推**
2. **TCS 过暗** → 颜色判断不可靠
3. **超时太长（15s）** → 即便推不动也消耗大量时间
4. **`push_duty=66` vs `push_correct_duty`** 配置不当，导致陷入"推不动→反向→推"的死循环

**修复建议**
- 检查 `config.py` 的 `push_duty` / `push_correct_duty` 是否合理
- `_push_watch_frame` 中 `lost_frames` 阈值过宽（默认 30 帧 ≈ 3s）
- 推动时若检测到 stuck（duty 满但 yaw 不变），应更快 skip

---

## 🟠 问题 4：HUNT lost 频繁（25 次旋转事件）

**症状**
```
[MATCH] HUNT lost → reverse spin
[MATCH] HUNT SPIN → TRACK    (频繁切换)
[MATCH] HUNT head cls=X → end, new head=Y    (目标轮换)
```

**根因**
1. **`match_hunt.py:208` `lost_frames` 阈值过松** — 一次目标丢失就反转
2. **`pick_class_timeout_ms` 让车辆频繁换类**（行 238-244）— 见下面问题 6
3. **`reverse_angle` 反转累积角度**（行 188-192），但 `rev_acc` 单调累加，遇到小幅来回抖动也累加，导致提前退出
4. **IMU yaw 不稳** → `search_speed` 开环旋转期间 yaw 漂移，无法积累可信的角度反馈

**典型死循环**（看行 296-340 区间）
- SPIN 找 → 一闪而过 → TRACK → 立刻 lost → 反转 SPIN → 又 TRACK → lost → 反转……

**修复建议**
- 区分"目标瞬时丢失"和"目标真不在视野里"——后者才触发反转
- `reverse_angle` 改成 `reverse_arc`（基于 IMU yaw 差累加）而不是基于每帧 yaw 增量
- `pick_class_timeout_ms` 适当加大（>10s），避免还没看完一类就跳到下一类

---

## 🟡 问题 5：ALIGN lost 频繁（15 次）

**症状**
```
[MATCH] ALIGN lost → reverse find
[MATCH] ALIGN lost long → HUNT
```

**根因**
- `match_hunt.py:250-272` `_align_lost_soft` 设计是"短丢等 → 中丢反转 → 久丢回 HUNT"
- 但 **TCS I2C 失败率高** → 在 ALIGN 阶段若车身压到 TCS 干扰区 → 黄线抖动 → ISR 触发 BACKOFF → 进入 BACKOFF 后又因 TCS 不稳退到 HUNT → 来回切

**特别看行 588-600**
```
yaw=-59.3 tgt=-60.0 err=-0.4  ← 进入 ALIGN
... lost 累积到 12 ...
yaw=-119.8 tgt=-60.0 err=+59.8  ← yaw 偏离 60°，目标又找到
```
在 60° 范围内找球，已经算效率不错了。但 **进入下一轮 HUNT 后又开始旋转找**。

---

## 🟡 问题 6：CAM filtered 18 次

**症状**
```
[CAM] filtered n=1 cls=1 sc=23 want=0 allow=None    ← 想找0类但只看到1类
[CAM] filtered n=1 cls=0 sc=25 want=1 allow=None    ← 想找1类但只看到0类
```

**根因**（看 `main.py:163-170`）
```python
if raw_n > 0 and ticks_diff(now, _filt_ms) > 1000:
  _filt_ms = now
  ...
  info("CAM", "filtered n=%d cls=%d sc=%d want=%s allow=%s" % ...)
```

这是 `match.filter_class` (单类过滤) 和 `match.match_allow` (多类允许) 的作用。当前比赛期望顺序是 `match_order`（如 `[1, 2, 0]`），找完一类才允许下一类。

**问题**：当 `_remaining = [1, 2, 0]` 时，`match_allow=[1, 2, 0]`，**全允许**，不会被过滤。只有当 `filter_class != 7` 时（即已锁定某一类），其他类才会被过滤。

但 log 频繁出现 `want=0 allow=None` → 找 0 类但目标其实是 1 类。这说明：
- **`_lock_active_class` 在 HUNT 阶段就已经锁了某一类**，但下一类还没出现在视野里
- 等了 1 秒还没看到 → 触发 `pick_class_timeout_ms` → 换类（见行 241-244）

**修复建议**：在 IDLE 阶段，camera 输出的所有类都应记录，作为"还剩哪些类没找"的预判依据。

---

## 🟡 问题 7：IMU yaw 漂移

**症状**（看 `[CAL]` 行）
- 行 77：`yaw=-3.2 mrel=-2.5 mag=(-220,708,64)`
- 行 82：`yaw=-2.2 mrel=-22.3 mag=(-420,558,163)` — **mrel 跳 -20°**
- 行 117：`yaw=-34.7 mrel=-97.1 mag=(-641,-261,-105)` — 大跳变
- 行 254：`mag=(-1661,-1616,2265)` — 电机电流磁场畸变 ±1600

**根因**
1. **磁力计受电机/电源干扰严重** — 看 `mag` 数据，电机一开就抖
2. `imu.py` 的 mag pull / dead 处理可能不够 — 看看 `cfg.imu_mag_dead`、`cfg.imu_mag_pull_max` 配置
3. **HUNT 旋转期间没有屏蔽磁力计** — 开环旋转时 yaw 应当纯靠陀螺仪

**修复建议**
- 加大 `cfg.imu_mag_pull_max`（或减低）— 当前可能让 mag 干扰了 yaw 估计
- 在 `_tick_hunt_spin` 期间给 IMU 加 `motor_on=True` 标志强制只用 gyro

---

## 📊 关键时序数据

| 阶段 | 实际耗时 | 主要耗时原因 |
|------|----------|--------------|
| LEAVE → 第一个球 | ~7s | IMU yaw 漂移 + LEAVE timeout |
| 找第一个球 (HUNT) | ~12s | 旋转找 + CAM filtered |
| 第一个球 → BACKOFF | ~2s | yellow ISR 正常触发 ✅ |
| 第二个球寻找+对齐+推动 | **~50s** | PUSH 超时 15.7s + ALIGN 反复 lost + HUNT 反转 |
| 第三个球寻找+对齐+推动 | **~70s** | 多次 ALIGN lost long → HUNT |
| HOME | ~15s | LEG1 走太久，没立刻触发黄线 |

**总比赛时长 ≈ 215s**（理想情况应在 90s 内）。

---

## 🎯 优先修复建议（按 ROI 排序）

1. **【最优先】修 TCS I2C**：加外部上拉电阻到 C19/C18，是最可能立刻见效的改动
2. **修主循环 dt**：检查 `match.tick` 内部是否有阻塞调用（特别是 GC、`info()` 内的字符串构造）
3. **PUSH 超时**：把 `push_timeout_ms` 从当前值降到 8000ms，推动失败更快跳过
4. **HUNT lost 阈值**：把 `lost_frames` 从默认调到 6~8（默认看起来过松）
5. **pick_class_timeout_ms**：从当前值加大到 15000ms，避免过早换类
6. **IMU mag 抑制**：HUNT 旋转期间禁用 mag pull，纯靠 gyro

## 📝 上下文信息

- 比赛参数：`layout=2 N=3 duty=50`
- 进球顺序：cls=0 → cls=1 → cls=2（最终 rem=[0,1,2] 全清空）
- HOME 分两段：LEG1_DRIVE 到 +120°，BACKOFF_TURN 到 -150°，LEG2_DRIVE 完成
- 全部得分均由 `[MATCH] yellow → BACKOFF (ISR)` 触发，**没有一次是正常 PUSH → 黄线 → SCORE**

> 这意味着 PUSH 阶段从未真正把球推到位，得分全靠超时后的 ISR 误触发或车头越界兜底