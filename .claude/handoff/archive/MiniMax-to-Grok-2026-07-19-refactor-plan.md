# MiniMax → Grok 综合重构方案（完整版）

> 日期：2026-07-19
> 来源：4 阶段 Workflow（审计→设计→对抗审查→综合）
> 骨架：B 方案（稳健优先，评分 B+）；不采纳 A（激进，D）与 C（模块化，C-）

---

## 0. 综述

**核心发现（综合阶段新诊断）**：log 138-201 显示 PUSH 失败模式不是"timeout 不够长"或"盲推距离不够"，而是 **"进入 PUSH 后 0.5s 即因丢帧/过偏 reseek"**（`PUSH → CORRECT (skew)`）。`_push_bad` 双语义（lost+skew 共用一个计数器，切 kind 清零 → 累计丢失）+ 10FPS 下进入瞬间 200ms 盲区被当成丢失。

**方案核心**：
- 骨架 B（枚举化 + 文件拆分方向）
- 不采纳 B2 全局 MatchState 扁平枚举（`split()` + `@property` 在 200Hz 热路径慢）
- 不采纳 B1.2 YawRef 类封装（`_enter_push:82` 已 `_hold_yaw=_yaw()`，污染已缓解）
- 不采纳 C 方案 PhaseController 抽象（768KB GC 风险、迁移路径不明）
- 新增 A0 "PUSH 进入宽限期"（最高 ROI 修复）
- (7) 提前 PUSH 降级为"机制落地、默认保守"

---

## S1. 综合后的文件结构

纯 mixin 组合（`MatchRunner` 多继承各 mixin，运行时 `self.` 解析），不产生 B 方案担心的 import 循环：

```
match_isr.py ──► match_push.py ──► match_hunt.py ──► match.py (MatchRunner)
(ISR/BACKOFF/    (PUSH mixin)      (HUNT+ALIGN)       组合所有 mixin
 boundary enum)                                       + HOME + 调度 + 基础写电机
```

| 文件 | 现状 | 目标 | Δ | 承载 |
|---|---:|---:|---:|---|
| `match.py` | 563 | ~568 | +5 | `MatchRunner.__init__`(补默认值)、`start/stop/tick/_fault`、基础写电机、HOME、调度、枚举常量 |
| `match_hunt.py` | 558 | ~442 | -116 | HUNT+ALIGN（同文件，因共享 `_search_dir`/`_align_lost_soft`）+ 队首 + `_push_yaw` + 边界武装 |
| `match_push.py` | — | ~185 | +185 | **新**：PUSH 全部 + 宽限期/passed/micro |
| `match_isr.py` | 199 | ~191 | -8 | ISR 场锁(boundary enum)、BACKOFF、`YellowHit` 封装 |
| `config.py` | 443 | ~475 | +32 | 6 个新参数（field + to_dict + _set_one 三处） |
| `main.py` | 427 | 427 | 0 | 不动 |
| **合计** | **1763** | **~1861** | **+98 (+5.6%)** | 远低于 +15% 上限 |

### 字段/方法迁移落点

| 来源 | 去向 | 说明 |
|---|---|---|
| `match_hunt.py:78-558` 所有 `_push_*` 方法 | `match_push.py` (MatchPush mixin) | 纯剪切；`self.` 调用运行时解析 |
| `_push_bad`/`_push_bad_kind` | 删除 → `_push_lost_n`/`_push_skew_n`/`_push_last_kind` | D1 解耦 |
| `_search_dir` | 拆 `_hunt_search_dir`/`_align_search_dir` | D2 解耦 |
| `_search_phase`/`_rev_acc` | **删除** | D3 死字段 |
| `_yellow_hit`+`_yellow_hit_phase` | 合并为 `YellowHit` 对象 | D4 封装 |
| `_boundary_armed/pending/need_cross` 3 bool | 合并为 `_boundary` enum | D5 |
| `_push_seen/_push_last_cx/_push_last_y2/_push_slipped/_push_passed/_push_stuck_n/_push_micro_until` | 全部在 `match.py __init__` 显式赋默认值 | D6 |

---

## S2. 解耦修复清单（必做）

| # | 修复点 | 修复方法 | 改动位置 | LOC |
|---|---|---|---|---:|
| D1 | **`_push_bad` 双语义**：lost/skew 共用 + 切 kind 清零 | 拆 `_push_lost_n`/`_push_skew_n`/`_push_last_kind`，切换"归 1 不归 0" | `match_push.py:_push_watch_frame` 重写 | +8 |
| D2 | **`_search_dir` HUNT/ALIGN 共享污染** | 拆 `_hunt_search_dir`/`_align_search_dir` | `match.py __init__`、`match_hunt.py` 各写点 | +3 |
| D3 | **死字段** `_search_phase`/`_rev_acc` | 直接删除 | `match.py __init__`、`match_hunt.py` 所有写点 | -6 |
| D4 | **`_yellow_hit`+`_yellow_hit_phase` 双字段信封** | 封装 `YellowHit` 类：`fire/consume/pending` 单点管理 | `match_isr.py` | +6 |
| D5 | **`_boundary_*` 3 bool = 1 个 FSM** | 合并 `_boundary` enum（IDLE/PENDING_CROSS/PENDING_OFF/ARMED），触发瞬态仍由 `YellowHit` | `match_isr.py:isr_field_lock`、`match_hunt.py:_arm_boundary_when_clear`、`match.py:_enter_home/start/stop/_fault` | -8 |
| D6 | **PUSH 字段隐式初始化** | `__init__` 显式赋默认值 | `match.py __init__` | +5 |
| D7 | **`_sub` 6 套 FSM 命名冲突** | **轻量版**：仅重命名 `BACKOFF/SPIN→BO_SPIN`、`HOME/BACKOFF→H_BACKOFF`（不做 B2 全局 MatchState） | `match_isr.py:step_backoff`、`match.py:_home_*` | +2 |

**对 B 骨架的 2 处偏离**：
1. **不采纳 B2 选项 C**（`split()`+`@property` 在 200Hz 热路径慢，且 `status_text` 兼容性风险）
2. **不采纳 B1.2 YawRef**（`_enter_push:82` 已 `_hold_yaw=_yaw()`，污染已缓解；类封装零收益 +12 LOC）

---

## S3. 激进策略落地清单（按 ROI：7 > 1 > 3 > 2 > 4）

### A0 —— PUSH 进入宽限期【最高 ROI，必做】

**修复"进入即丢帧 reseek"**。

```python
# match_push.py: _push_watch_frame 开头，t is None 分支之前
if t is None:
    if elapsed < int(self._cfg.push_entry_grace_ms):
        return None   # 进入宽限期内丢帧不计数
```

- 配置 key：`推箱进入宽限ms`（`push_entry_grace_ms`，默认 **400**）
- 回退：设 `0` = 完全等价旧逻辑
- LOC：+4

### (7) 提前 PUSH —— y2 阈值参数化【机制落地，默认保守】

```python
def _hunt_arrive_y2(self):
    tr = self._cfg.tracking
    if getattr(self._cfg, "match_mode", "final") != "pre":
        buf = float(getattr(self._cfg, "early_push_y2_buf", 5.0))
        return min(float(tr.stage_bottom_pct), float(tr.contact_bottom_pct) - buf)
    return float(tr.stop_bottom_pct)
```

- 配置 key：`提前PUSH缓冲`（`early_push_y2_buf`，**默认 5 = 等价现状**）
- 回退：设 `5` 即当前行为
- 审查证明激进值会加剧 skew（log line 195 显示 CORRECT 主要失败模式），故默认保守
- LOC：+3

### (1) 盲推放宽 —— `_push_passed` latch【落地】

```python
def _push_occlusion_ok(self):
    if self._push_slipped or self._sub == "CORRECT":
        return False
    if not self._push_seen or not self._push_cx_ok(self._push_last_cx):
        return False
    elapsed_s = ticks_diff(ticks_ms(), self._phase_ms) / 1000.0
    if elapsed_s * float(self._cfg.push_duty) >= float(self._cfg.push_pass_units):
        self._push_passed = True
    thr = (float(self._cfg.tracking.contact_bottom_pct) - float(self._cfg.push_passed_y2_buf)
           if self._push_passed else float(self._cfg.tracking.stage_bottom_pct))
    return self._push_last_y2 >= thr
```

- 字段：`_push_passed`（`__init__`+`_enter_push` 均置 False）
- 配置 key：`推箱穿越积分阈`（`push_pass_units`，**默认 360 = duty12×3s，无量纲时间积分非物理厘米**——回应审查"公式无校准"指摘，明确为可调经验阈）、`推箱穿越后y2放宽`（`push_passed_y2_buf`，默认 8）
- 回退：`push_pass_units=99999` = 永不 latch = 旧逻辑
- LOC：+10

### (3) 多次轻推 —— MICRO_BACKOFF sub【落地，默认关闭】

- 触发：DRIVE 中连续 `push_stuck_frames` 帧 y2 不增（卡墙）→ 微退 `push_micro_ms`（duty=`-push_duty×0.5`）→ 回 DRIVE
- 因 `push_timeout_ms=3000` 极紧，最多 1 次（`_push_micro_count ≤ 1`）
- 字段：`_push_stuck_n`、`_push_micro_until`(ms)、`_push_micro_count`、`_push_y2_prev`
- 配置 key：`推箱卡死帧数`（`push_stuck_frames`，默认 4）、`推箱微退ms`（`push_micro_ms`，默认 200）、`推箱微退开关`（`push_micro_enable`，**默认 false**）
- **关键约束**：MICRO_BACKOFF 期间不复用 `_backoff_busy`（ISR-BACKOFF 专用）；压黄线仍让 ISR 正常 BACKOFF（不破坏场锁时序）
- LOC：+18

### (2) 跳 ALIGN / (4) 预测推 —— 仅留扩展点

- (2)：现状 `_on_hunt_arrived:180` 已有 `if ty is None:` 直进 PUSH 分支，保留分支不扩展；配置位 `skip_align_enable`（默认 false）预留
- (4)：无里程 API、IMU 陀螺积分 ±5° drift，**不实现**
- LOC：+1

**S3 激进部分合计 LOC**：+4+3+10+18+1 = **+36**

---

## S4. 实施 PR 切片

| PR | 标题 | 文件 | LOC | 独立回滚验证 |
|---|---|---|---:|---|
| **PR1** | `__init__` 补 PUSH 字段默认值（D6） | match.py | +5 | 启动无 `AttributeError` |
| **PR2** | 删死字段 + 拆 `_search_dir`（D2,D3） | match.py, match_hunt.py | -3 | grep 确认 `_search_phase`/`_rev_acc` 零引用 |
| **PR3** | `YellowHit` 封装（D4） | match_isr.py | +6 | 压黄线→BACKOFF 触发一次且仅一次 |
| **PR4** | 场锁 `_boundary` enum（D5） | match_isr.py, match_hunt.py, match.py | -8 | **重点**：3 个不变契约现场验证 |
| **PR5** | `_sub` 冲突重命名（D7） | match_isr.py, match.py | +2 | BACKOFF/HOME sub 命名区分 |
| **PR6** | 析出 `match_push.py` | match_hunt.py, match_push.py | +69 | 真机 `free` ≥ 主线；逐帧回归 |
| **PR7** ⭐ | PUSH 计数解耦 + 进入宽限期（D1 + A0） | match_push.py, config.py | +16 | log 138-201 回放验证 |
| **PR8** | 激进 (7)+(1) | match_push.py, match_hunt.py, config.py | +17 | 默认参数=主线 |
| **PR9** | 激进 (3) MICRO_BACKOFF | match_push.py, config.py | +18 | 默认关闭；开启后 ≤1 次微退 |

**依赖链**：PR1→PR2→PR3→PR4→PR5（零行为变化）→PR6（拆分）→PR7（行为变化核心）→PR8-9（激进）

**合计**：+122 LOC（+6.9%）

---

## S5. 验证方法

| PR | 验证 |
|---|---|
| PR1 | 真机 REPL `import match`；回归：跑主线 sensors 序列对比 `navigation_snapshot` |
| PR2 | `grep -rn "_search_phase\|_rev_acc" CODE/` 应为空；现场 HUNT 2 圈观察 `flip spin dir` |
| PR3 | 压黄线 3 次，`MATCH backoff` 事件=3 |
| PR4 | **关键**：①冷启动不压起始线不触发 ②压线离线后武装 ③3s 兜底 ④进场压黄线立即 BACKOFF |
| PR5 | BACKOFF 阶段 NAV `sub=BO_SPIN`、HOME `sub=H_BACKOFF`；`status_text` 上位机解析兼容 |
| PR6 | 真机 `gc.mem_free()` ≥ 主线；同一 sensors 序列下 `navigation_snapshot` 完全一致 |
| PR7 | **回放 log 138-201**：进入 PUSH 后 400ms 内**不**产生 `PUSH reseek`；构造 skew→lost→skew 序列断言 `_push_skew_n` 不被清零 |
| PR8 | A/B 现场：`early_push_y2_buf=5` vs `10` 各跑 10 轮；`push_pass_units` 同法 |
| PR9 | 人为抵墙制造卡死，观察 ≤1 次 `MICRO_BACKOFF`；微退中压黄线须走正常 ISR BACKOFF |

**同步补的可观测性**（P0 缺口）：NAV 加 `push_lost_n / push_skew_n / push_passed / grace`，CAM 加 `raw_n/filtered/allow`——约 +8 LOC，可并入 PR7。

---

## S6. 风险与回退

| 严重度 | 风险 | 回退 |
|---|---|---|
| **高** | PR6 拆分撞 768KB / GC 抖动 | 真机测 `mem_free`；逼近 `_GC_LOW` 回滚 PR6，PR7-9 直接落 `match_hunt.py` |
| **高** | PR4 场锁 enum 破坏 BACKOFF 时序 | enum 三态转移逐条对照 S5 PR4 四项；任一不符 `git revert` PR4 |
| **中** | `_push_passed` latch 残留 | `_enter_push` 强制 `_push_passed=False`；回退 `push_pass_units=99999` |
| **中** | 提前 PUSH 加剧 skew | 默认 `early_push_y2_buf=5`=现状；激进值需 PR7 验证稳定 |
| **中** | MICRO_BACKOFF 吃掉 3s 预算 | `_push_micro_count≤1` + `push_micro_enable=false` 默认关 |
| **低** | `status_text` 字符串变更影响上位机 | HUNT/ALIGN/PUSH 对外字符串不变；旧映射别名 |
| **低** | A0 宽限期过长掩盖真丢失 | 设 0 回退；超时后正常计数 |

---

## 一页速览

```
解耦(零行为变化)              拆分            行为变化(激进默认保守/关)
PR1 补默认值       ─►                ─►
PR2 删死字段/拆dir ─►  PR6 析出      ─►  PR7 计数解耦+进入宽限(A0)  ★真实bug修复
PR3 YellowHit      ─►    match_push  ─►  PR8 提前PUSH(7)+盲推(1)   默认=现状
PR4 场锁enum       ─►                ─►  PR9 MICRO_BACKOFF(3)      默认关闭
PR5 sub重命名      ─►
                                          (2)跳ALIGN / (4)预测推 = 仅留扩展点
```

---

## 附录：对抗审查要点

- **方案 A 评级 D**：3 个致命漏洞（BLIND_PUSH 距离公式无校准；`_push_bad` 拆分未解决"进入即丢帧"；PUSH 新 sub 与场锁 ISR 同步未明）
- **方案 B 评级 B+**：无致命漏洞；但漏诊了 PUSH 真实失败模式（"过偏即丢帧"被当成 skew）
- **方案 C 评级 C-**：3 个致命漏洞（迁移路径不明；3 模块撞 768KB GC；Policy 接口内存开销未量化）

**总评**：骨架取 B 的枚举化/PR 切片方向；**关键增值**是采纳对抗审查的证据，把"PUSH 进入宽限期(A0)+skew 计数解耦(D1)"定为真实失败根因的头号修复，并据此把 (7) 提前 PUSH 从"激进默认"降级为"保守默认、机制就绪"。原则 4（BACKOFF 场锁时序）由 PR4 四项现场验证守护，原则 5（相机/IMU/TCS 驱动）零改动。