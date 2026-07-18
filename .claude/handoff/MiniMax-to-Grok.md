# MiniMax → Grok

> 完整重构方案：[`archive/MiniMax-to-Grok-2026-07-19-refactor-plan.md`](archive/MiniMax-to-Grok-2026-07-19-refactor-plan.md)
> 数据源：`CODE/log.txt`（935 行 + 之前 2747 行） + 4 阶段 Workflow（审计→设计→审查→综合）
> 总改动：**+122 LOC（+6.9%，上限 +15%）**，骨架取稳健方案 B+，激进策略按 ROI 分级落地。

---

## 待 Grok 确认（按 PR 顺序，每片可独立回滚）

| PR | 标题 | 文件 | LOC | 关键变更 |
|----|------|------|----:|----------|
| **PR1** | `__init__` 补 PUSH 字段默认值 | `match.py` | +5 | 消除 `_push_seen/_push_last_cx/_push_last_y2/_push_slipped` 隐式初始化脆弱契约 |
| **PR2** | 删死字段 + 拆 `_search_dir` | `match.py`, `match_hunt.py` | -3 | 删 `_search_phase`/`_rev_acc`；`_search_dir` → `_hunt_search_dir`+`_align_search_dir`（解 HUNT/ALIGN 共享污染） |
| **PR3** | `YellowHit` 封装 | `match_isr.py` | +6 | `_yellow_hit`+`_yellow_hit_phase` 双字段 → `YellowHit.fire/consume/pending` 单点管理 |
| **PR4** | 场锁 `_boundary` enum | `match_isr.py`, `match_hunt.py`, `match.py` | -8 | 3 bool → enum（IDLE/PENDING_CROSS/PENDING_OFF/ARMED）；**红线**：3s 兜底、saw_line 必压、ARMED+on_line 触发三条不变 |
| **PR5** | `_sub` 冲突重命名 | `match_isr.py`, `match.py` | +2 | BACKOFF 的 `SPIN` → `BO_SPIN`，HOME 的 `BACKOFF` → `H_BACKOFF`（避免与 HUNT `SPIN`、phase `BACKOFF` 重名错位） |
| **PR6** | 析出 `match_push.py` | `match_hunt.py` → `match_push.py` | +69 | PUSH 全部剪到新文件，HUNT/ALIGN 同文件（避免 B 方案担心的 import 循环） |
| **PR7** ⭐ | PUSH 计数解耦 + **进入宽限期** | `match_push.py`, `config.py` | +16 | **`_push_bad` 拆 `_push_lost_n`/`_push_skew_n`（修切 kind 清零 bug）** + 进入 PUSH 后 400ms 内丢帧不计（修"进入即丢 2 帧误 reseek"） |
| **PR8** | 激进 (7)+(1)：提前PUSH+盲推 | `match_push.py`, `match_hunt.py`, `config.py` | +17 | `early_push_y2_buf=5`(默认=现状) / `push_pass_units=360`；激进值需现场试 |
| **PR9** | 激进 (3) MICRO_BACKOFF | `match_push.py`, `config.py` | +18 | 卡墙微退 200ms，默认 `push_micro_enable=false`；微退中压黄线仍走正常 ISR BACKOFF |

**PR1-6 零行为变化**（纯解耦）；**PR7** 是核心 bug 修复；**PR8-9** 激进开关默认保守/关。

---

## 关键裁决（综合阶段新发现 vs 旧 MiniMax-to-Grok）

| 旧 handoff 提的 | 新方案对应 | 状态 |
|-----------------|-----------|------|
| PUSH skew 纠偏 timeout 加 4s 上限 | **PR7 不延长而是解耦** — skew 是真 bug 根因，延超时只是治标 | **升级为 PR7 优先** |
| HUNT 换类加前置条件 | PR8 间接修 — `early_push_y2_buf` 调低让 HUNT 早退 ALIGN；换类逻辑留作后续 PR | 后续 |
| BACKOFF 完强制回场心 | 不在本次重构范围 | 后续 |
| dt 钳位 0.5→0.15 | 不在范围（属 main.py GC 优化） | 后续 |
| HUNT_SPIN 加 6 圈硬上限 | 不在范围 | 后续 |

**核心反转**：旧 handoff 假设 PUSH 失败是"timeout 不够长"；新诊断显示**是"进入即丢帧 reseek"**——`_push_bad` 双语义导致切 kind 清零 + 进入瞬间盲区被当成丢失。

---

## 落地建议

1. **先做 PR1-6**（纯解耦，2-3 小时），零风险可一次推完
2. **PR7 是必做的核心修复**（半天），需现场跑 log 回放验证
3. **PR8-9 激进开关**——先 PR8 把参数化落地（默认保守），跑 3-5 轮观察；激进值（`early_push_y2_buf=10`/`push_micro_enable=true`）需带防护（关黄线后做）

## 期望效果

- PUSH 进入即 reseek 误触发 → 修
- 激进策略参数化后现场可调 → 不再硬编码"30cm"
- 6 个耦合点全部类型化 → 下次加新策略只动 match_push.py 即可
- 字段、enum、sub 重命名冲突 → 0 重名

## 验收要点（每 PR）

| PR | 验收 |
|----|------|
| PR4（场锁） | **重点**：①冷启动不压起始线不触发 ②压线离线后武装 ③3s 兜底（遮 TCS 模拟） ④进场压黄线立即 BACKOFF |
| PR6（拆分） | 真机 `gc.mem_free()` ≥ 主线基线；同一 sensors 序列下 `navigation_snapshot` 完全一致 |
| PR7（核心） | 回放 log 138-201 原始丢帧序列，进入 PUSH 后 400ms 内**不**产生 `PUSH reseek`；构造 skew→lost→skew 序列断言 `_push_skew_n` 不被清零 |