# MCU 端完整流程图与状态机

> 范围：上电启动 → 主循环 → `MatchRunner` 全相态（含 ISR 场锁与 BACKOFF 原子步进）。
> 代码：`CODE/main.py` + `CODE/match.py` + `CODE/match_hunt.py` + `CODE/match_isr.py`。

---

## 0. 总览：分层结构

```
┌──────────────────────────────────────────────────────────────────┐
│ Layer 1: main.py 主循环（无状态机，单次 while True）             │
│   phase ∈ {IDLE, RUN, DONE, FAULT}                              │
│   每帧: dt clamp → C20 按钮 → _build_sensors → match.tick → LED  │
└────────────────┬─────────────────────────────────────────────────┘
                 │
┌────────────────▼─────────────────────────────────────────────────┐
│ Layer 2: match.tick() 派发（match.py）                          │
│   phase ∈ {IDLE, LEAVE, HUNT, ALIGN, PUSH, HOME, BACKOFF,       │
│           DONE, FAULT}                                          │
│   顺序: flush_deferred → consume_yellow_hit → _post_backoff →   │
│         step_backoff → 分相 tick                                 │
└────────────────┬─────────────────────────────────────────────────┘
                 │
┌────────────────▼─────────────────────────────────────────────────┐
│ Layer 3: 各 phase 子状态机                                       │
│   HUNT  → {SPIN, FWD, TRACK}                                    │
│   ALIGN → {TURN, CLOSE}                                         │
│   PUSH  → {DRIVE, CORRECT}                                      │
│   BACKOFF→{RETREAT, SPIN, DONE}                                 │
│   HOME  → {LEAVE_LINE, LEG1_TURN, LEG1_DRIVE,                   │
│           BACKOFF, BACKOFF_TURN, LEG2_DRIVE}                    │
└──────────────────────────────────────────────────────────────────┘
┌──────────────────────────────────────────────────────────────────┐
│ 并行 Layer: ISR 50Hz 场锁（match_isr.isr_field_lock）            │
│   _boundary_pending → _boundary_armed → _yellow_hit → 主消费    │
└──────────────────────────────────────────────────────────────────┘
```

---

## 1. 顶层主循环 phase 状态机（`main.py`）

```
              ┌──────────────────────────┐
              │      IDLE (长亮 LED)    │
              │  短按 C20 <2s 启动       │
              └────────────┬─────────────┘
                           │ match.start() 成功
                           │ tkr_tcs.start(20)  启动 50Hz TCS+场锁
                           ▼
              ┌──────────────────────────┐
              │       RUN                │◄────────────┐
              │  match.tick(dt, sensors) │             │
              │  match.phase != IDLE     │             │
              └──┬────────┬──────────┬───┘             │
                 │        │          │                 │
   match.phase=  │   match.phase=   │  match.phase=   │
   DONE          │   FAULT          │  IDLE（被 stop）│
                 ▼        ▼          ▼                 │
        ┌──────────┐ ┌────────┐ ┌──────────┐           │
        │  DONE    │ │ FAULT  │ │ (回 IDLE)│           │
        │ 闪 LED   │ │ 双闪   │ │  长亮    │           │
        │ 再按 C20 │ │ 按 C20 │ │ 再按 C20 │           │
        │ → IDLE   │ │ → IDLE │ │ 启动     │           │
        └─────┬────┘ └───┬────┘ └─────┬────┘           │
              │         │            │                 │
              └─────────┴────────────┴───→ IDLE ───────┘
```

**进入 RUN 条件**：`phase ∈ (IDLE|DONE)` ∧ C20 短按(<2s) ∧ IMU 校准成功 ∧ 相机就绪 ∧ `match.start()` 返回 True。
**RUN→DONE**：`match.phase == "DONE"`（已 `scored_count >= match_target_count`）。
**RUN→FAULT**：`match._fault(...)` 被任意路径调用（IMU/相机超时/HOME timeout/cam timeout in HUNT/无目标类可分）。
**任意→IDLE**：C20 短按触发 `match.stop()`。
**C20 长按 ≥2s**：忽略（无 stop）。

---

## 2. MatchRunner.phase 状态机（`match.py::tick`）

```
        ┌──────────────────────────────────────────────┐
        │ IDLE ──start()──► LEAVE ──见目标/超时──► HUNT│
        │                          │                  │
        │                          └─超时(timeout)──► │
        └──────────────────────────────────────────────┘
                                   │
                                   ▼
   ┌───────────────────────────────────────────────────────┐
   │   HUNT ◄──HUNT timeout / ALIGN timeout /              │
   │     │     ALIGN lost long / PUSH timeout /             │
   │     │     PUSH reseek / _skip_or_home                  │
   │     ▼                                                  │
   │   ALIGN  (已锁定 cls, 需要航向)                        │
   │     │                                                  │
   │     │  y2≥stage ∧ yaw_ok ∧ cx_ok 持续 N 帧            │
   │     ▼                                                  │
   │   PUSH                                                  │
   │     │                                                  │
   │     ├──► BACKOFF ──► HOME ──► DONE                    │
   │     │                                                  │
   │     ├──► reseek ──► HUNT (reverse=True)                │
   │     └──► timeout / boundary ──► _skip_or_home          │
   └───────────────────────────────────────────────────────┘
```

**进入各 phase 的唯一入口：**

| 目标 phase | 调用者 | 入口函数 | 行号 |
|-----------|--------|---------|------|
| LEAVE | `start()` | `self.phase = "LEAVE"` | match.py:155 |
| HUNT(SPIN/FWD/TRACK) | `_enter_hunt(reverse, tracking, forward)` | `self.phase = "HUNT"` | match_hunt.py:11 |
| ALIGN(TURN) | `_enter_align(target_yaw)` | `self.phase = "ALIGN"` | match_hunt.py:114 |
| PUSH(DRIVE) | `_enter_push()` | `self.phase = "PUSH"` | match_hunt.py:78 |
| BACKOFF(RETREAT→SPIN) | `_start_backoff()` | `self.phase = "BACKOFF"` | match_isr.py:100 |
| HOME | `_enter_home()` | `self.phase = "HOME"` | match.py:419 |
| DONE | `_finish()` 或 `_home_leg2_drive` 压线 | `self.phase = "DONE"` | match.py:529 |
| FAULT | `_fault(why)` | `self.phase = "FAULT"` | match.py:344 |

**退出条件汇总：**

| From → To | 触发条件 | 代码位置 |
|-----------|---------|---------|
| LEAVE → ALIGN | 见到目标 ∧ `y2 ≥ stage-5` ∧ 有 push_yaw 偏角 | match_hunt.py:145-148 |
| LEAVE → HUNT(TRACK) | 见到目标 ∧ 上述条件不满足 | match_hunt.py:149-151 |
| LEAVE → HUNT(SPIN) | `drive_timeout_ms` 到时未见到目标 | match_hunt.py:153-156 |
| HUNT → ALIGN | TRACK 中 `y2 ≥ hunt_arrive_y2()` ∧ 有 push_yaw | match_hunt.py:307-313 |
| HUNT → PUSH | TRACK 中 `y2 ≥ contact` ∧ `cx ∈ [cx_left_min, cx_right_max]` ∧ 无 push_yaw | match_hunt.py:181-184 |
| HUNT → HUNT(reverse) | TRACK 丢失 `lost_frames` 帧 | match_hunt.py:319-320 |
| HUNT → HUNT(SPIN flip dir) | SPIN 转满一圈 | match_hunt.py:298-303 |
| HUNT → HUNT(TRACK) | SPIN/FWD 连续 `confirm_frames` 帧见目标 | match_hunt.py:251-259 / 279-289 |
| HUNT → _skip_or_home | `pick_timeout_ms` 总超时 | match_hunt.py:345-347 |
| HUNT → FAULT | `cam_timeout=True` | match_hunt.py:341-343 |
| ALIGN → PUSH | CLOSE 中 `y2 ≥ contact` ∧ `cx` 居中 | match_hunt.py:424-428 |
| ALIGN → ALIGN(CLOSE) | TURN 中 yaw_ok ∧ cx_ok 持续 `orbit_confirm_frames` | match_hunt.py:460-465 |
| ALIGN → ALIGN(TURN) | CLOSE 中 yaw_err 超 1.5×tol | match_hunt.py:418-423 |
| ALIGN → HUNT(reverse) | 目标丢 `orbit_lost_frames×4` 帧（久丢） | match_hunt.py:372-378 |
| ALIGN → HUNT(reverse) | `orbit_timeout_ms` 超时 | match_hunt.py:388-391 |
| ALIGN → _skip_or_home | `approach_cluster_timeout_ms` 总超时 | match_hunt.py:384-387 |
| PUSH → HUNT(reseek) | 丢 `push_watch_frames` 帧连续 | match_hunt.py:111-112, 487-504 |
| PUSH → PUSH(CORRECT) | cx 偏 `push_watch_frames` 帧连续 | match_hunt.py:511-521, 546-550 |
| PUSH → PUSH(DRIVE) | CORRECT 中 cx 居中 | match_hunt.py:551-554 |
| PUSH → _skip_or_home | `push_timeout_ms` 超时（**未得分**！） | match_hunt.py:537-541 |
| **PUSH → BACKOFF** | **黄线急停（ISR 触发）∧ `consume_yellow_hit()`** | match_isr.py:70-77, 80-98 |
| BACKOFF → POST_HOME/FWD | RETREAT≥`backoff_retreat_min_ms` ∧ 离线 ∧ SPIN 转满 `backoff_spin_deg` | match_isr.py:117-150 |
| BACKOFF → POST_HOME | `_want_home` 或 `scored_count >= match_target_count` | match_isr.py:158-160 |
| BACKOFF → POST_FWD | 上述不满足 | match_isr.py:162-164 |
| POST_HOME → HOME | `tick()` 见到 `_post_backoff == "HOME"` | match.py:186-195 |
| POST_FWD → HUNT(forward=True) | `tick()` 见到 `_post_backoff == "FWD"` | match.py:191-194 |
| HOME → DONE | LEG1_DRIVE 压线 ∧ 无 home_y2（layout 1/4） | match.py:476-480 |
| HOME → DONE | LEG2_DRIVE 压线（layout 2/3） | match.py:521-523 |
| HOME → FAULT | `home_timeout_ms` 超时 ∧ `gate not confirmed` | match.py:439-441 |

---

## 3. HUNT 子状态机（`match_hunt.py::_tick_hunt`）

```
              ┌────────────────────────────────────┐
   start()    │  HUNT  sub=SPIN（默认）            │
   ──────────►│    │                               │
              │    ├─确认 confirm_frames 帧见物 ───►│
              │    │                               │
              │    ├─HUNT FWD timeout ───► SPIN    │
              │    │                               │
              │    └─转满一圈 ──► SPIN flip dir    │
              └────────────────┬───────────────────┘
                               │ 见目标 confirm_frames 帧
                               ▼
              ┌────────────────────────────────────┐
              │  HUNT sub=TRACK                    │
              │   y2 ≥ hunt_arrive_y2() ──►        │
              │     ├ 有 push_yaw ──► ALIGN        │
              │     └ 无 push_yaw ∧ cx 居中 ──►PUSH│
              │                                    │
              │   丢 lost_frames 帧 ──► reverse    │
              │     (search_dir *= -1)             │
              └────────────────────────────────────┘
```

**入口**：`_enter_hunt(reverse, tracking, forward)`
- `reverse=False, tracking=False, forward=False` → sub=SPIN（默认扫圈）
- `reverse=True` → sub=SPIN + `search_dir *= -1`（反向扫圈）
- `tracking=True` → sub=TRACK（LEAVE 见到目标时直接进 TRACK）
- `forward=True` → sub=FWD（BACKOFF 后回中区）

**关键阈值（来自 cfg）**：
- `tracking.confirm_frames` — 帧数确认
- `tracking.lost_frames` — TRACK 丢失容忍
- `pick_timeout_ms` — HUNT 总超时 → `_skip_or_home`
- `center_fwd_ms` — FWD→SPIN 超时
- `tracking.search_speed` — 自旋角速度
- `tracking.approach_speed` — TRACK 前进速度
- `tracking.bearing_actuation_sign` — 横向 PID 极性

**queue 轮换**：`_hunt_queue_update` 维护 `_remaining: List[int]`
- 见某类 cls ≥ `pick_class_frames` 帧连续 → 把它插入队首
- 队首连续 `pick_class_frames` 帧没见到 → 移到队尾
- 仅 SPIN/FWD 时触发；TRACK 时锁类不轮换

---

## 4. ALIGN 子状态机（`match_hunt.py::_tick_align`）

```
   _enter_align(target_yaw)
   ─────────────────────────►┌──────────────────────────┐
                             │  ALIGN sub=TURN          │
                             │   航向不对 → 绕前方轴转    │
                             │     spin + slip + radial  │
                             │   航向已对 (yaw_ok):      │
                             │     全向平移居中 cx→50     │
                             │     orbit_confirm 累积     │
                             │   cx_ok ∧ yaw_ok 持续 N 帧 │
                             └────────────┬─────────────┘
                                          │
                                          ▼
                             ┌──────────────────────────┐
                             │  ALIGN sub=CLOSE          │
                             │   前近距离 (final_approach│
                             │     speed, 含侧移)        │
                             │   y2 ≥ contact ∧ cx_ok:   │
                             │     ──► PUSH              │
                             │   y2 ≥ contact ∧ cx 偏:   │
                             │     侧移纠偏不回绕行      │
                             │   yaw 超 1.5×tol:         │
                             │     ──► 回 TURN           │
                             └──────────────────────────┘
```

**三档超时（互不嵌套）**：
1. `approach_cluster_timeout_ms` — 总超时（从 _enter_align 起算）→ `_skip_or_home`
2. `orbit_timeout_ms` — 阶段超时 → `_enter_hunt(reverse=True)`
3. `orbit_lost_frames × 4` — 久丢目标帧数 → `_enter_hunt(reverse=True)`

**反向找物逻辑**（`_align_lost_soft`）：
```
vision_lost < orbit_lost_frames          → 刹停等待
vision_lost == orbit_lost_frames         → 翻转 search_dir，原地反转找
orbit_lost_frames ≤ vision_lost < ×4    → 继续反转找
vision_lost ≥ orbit_lost_frames × 4     → 回 HUNT(reverse=True)
```

---

## 5. PUSH 子状态机（`match_hunt.py::_tick_push`）

```
   _enter_push()
   ─────────────►┌───────────────────────────────┐
                 │  PUSH sub=DRIVE               │
                 │   pwm = push_duty (66%)       │
                 │   持 yaw = 当前 yaw           │
                 └────────┬──────────────────────┘
                          │ cx 偏 push_watch_frames 帧
                          │ (连续 inside push_cx_ok 失败)
                          ▼
                 ┌───────────────────────────────┐
                 │  PUSH sub=CORRECT (skew)      │
                 │   横向 PID 修正               │
                 │   bearing_pid.update          │
                 │   pwm = push_correct_duty     │
                 └────────┬──────────────────────┘
                          │ cx 重新居中
                          ▼
                 ┌───────────────────────────────┐
                 │  PUSH sub=DRIVE               │
                 └───────────────────────────────┘
```

**`_push_watch_frame` 每帧状态**：

| 帧状态 | 行为 |
|--------|------|
| 见目标 ∧ cx 居中 | `_push_bad=0` → `"ok"` |
| 见目标 ∧ cx 偏 | `_push_slipped=True`，累积 skew → 达 `push_watch_frames` → `"correct"` |
| 丢目标 ∧ `_push_occlusion_ok()` | 视为近端遮挡 → `"ok"` |
| 丢目标 ∧ elapsed ≥ `push_lost_blind_ms` | 同上 |
| 丢目标 ∧ 其他 | 累积 lost → 达 `push_watch_frames` → `"reseek"` → `_enter_hunt(reverse=True)` |

**PUSH 退出**：
- 推满 `push_timeout_ms`（未压黄线、scored=0）→ `_skip_or_home`
- 推丢 `push_watch_frames` 帧 → reseek 回 HUNT
- **ISR 触发黄线急停** → `_start_backoff`（唯一得分路径）

---

## 6. ISR 场锁状态机（`match_isr.py::isr_field_lock`，50Hz）

```
        ┌─────────────────────────────────────────────┐
        │  _boundary_pending=False                    │
        │  _boundary_armed=False                      │
        │  _field_entered=False  (默认，start() 初始化)│
        └──────────────┬──────────────────────────────┘
                       │ _arm_boundary_when_clear()
                       │ 进入 HUNT/ALIGN/PUSH 时调用
                       ▼
        ┌─────────────────────────────────────────────┐
        │  _boundary_pending=True                     │
        │  _boundary_need_cross=True                  │
        │  _boundary_saw_line=bool(tcs.on_line)       │
        └──────┬──────────────────────────┬───────────┘
               │                          │
   !_field_entered                  _field_entered
   (出库/库外)                      (已进场)
               │                          │
   见到黄线 _boundary_saw_line=True     离线即武装
               │                          │
               │                          ▼
               │              ┌───────────────────────┐
               │              │  _boundary_pending=   │
               │              │    False              │
               │              │  _boundary_armed=True │
               │              └───────────┬───────────┘
   离线 _boundary_saw_line=True          │
               │                          │
               ▼                          │
        _boundary_pending=False           │
        _boundary_need_cross=False        │
        _boundary_saw_line=False          │
        _boundary_armed=True  ────────────┤
        _field_entered=True               │
        _def_armed=True                   │
                                          │
                                          ▼
                            ┌──────────────────────────┐
                            │  _boundary_armed=True    │
                            │  ISR 每帧:               │
                            │    if on_line (黄线):    │
                            │      force_brake()       │
                            │      _yellow_hit=True    │
                            │      _yellow_hit_phase=  │
                            │        self.phase        │
                            │      _boundary_armed=    │
                            │        False             │
                            └──────────┬───────────────┘
                                       │
                                       ▼
                            ┌──────────────────────────┐
                            │  主循环 consume_yellow_  │
                            │  hit():                  │
                            │   if hit_phase=="PUSH"   │
                            │     _push_score_ready()? │
                            │       _credit_score()    │
                            │       _def_score=1       │
                            │     else _def_score=2    │
                            │   _start_backoff()       │
                            └──────────────────────────┘
```

**`_push_score_ready` 三个 AND 条件**（match_isr.py:181）：
1. `_tcs.on_line == True`（物理压线）
2. `elapsed ≥ 200ms`（不是刚起步就误压）
3. `|yaw_err(_hold_yaw)| ≤ align_tol_deg × 2`（方向稳定）

---

## 7. BACKOFF 原子步进（`match_isr.py::step_backoff`）

```
   _start_backoff()        主循环分支
   ──────────────► ┌─────────────────────────────┐
                   │  BACKOFF sub=RETREAT         │
                   │   写电机后退 duty (= -drive) │
                   │   _yaw_target = yaw + 180°   │
                   └──────────┬──────────────────┘
                              │
                              │ elapsed ≥ recover_backoff_ms
                              │   OR
                              │ elapsed ≥ backoff_retreat_min_ms
                              │   AND (not on_line)
                              ▼
                   ┌─────────────────────────────┐
                   │  BACKOFF sub=SPIN           │
                   │   原地转 180°               │
                   │   |yaw - spin_start| ≥       │
                   │     backoff_spin_deg  →done │
                   │   elapsed > 2000 → done     │
                   └──────────┬──────────────────┘
                              │ _finish_backoff(aligned)
                              ▼
                   ┌─────────────────────────────┐
                   │  _post_backoff 决策:         │
                   │    if _want_home 或          │
                   │       scored ≥ N:           │
                   │      post = "HOME"          │
                   │    else:                     │
                   │      post = "FWD"           │
                   │  _backoff_busy=False        │
                   │  self.phase 仍为 BACKOFF    │
                   │  tick 下一帧:               │
                   │    action="HOME" → _enter_  │
                   │      home()                 │
                   │    action="FWD"  → _enter_  │
                   │      hunt(forward=True)     │
                   └─────────────────────────────┘
```

**关键不变量**：BACKOFF 期间 `_backoff_busy=True`，所有 `_write_*` 都被 `if self._backoff_busy: return` 拦截——其他 phase 的 tick 不会干扰原子步进。

---

## 8. HOME 子状态机（`match.py::_tick_home`）

按 `start_layout` 不同，几何路径不同：

```
                       _enter_home()
                       ─────────────►
                  on_line?  ──Y──►  sub=LEAVE_LINE
                       │           (后退到离线)
                       │N
                       ▼
                  sub=LEG1_TURN
                  (spin_toward → y1)
                       │
                       │ yaw_err(y1) ≤ tol × 3 帧
                       ▼
                  sub=LEG1_DRIVE
                  (前进到压线)
                       │
                       ├─ 压线 ∧ y2 is None ──► _finish() ──► DONE
                       │
                       └─ 压线 ∧ y2 ≠ None ──► sub=BACKOFF (回退)
                                                  │
                                                  │ min_ms 过了 ∧ (离线 OR timeout)
                                                  ▼
                                            sub=BACKOFF_TURN
                                            (spin_toward → y2)
                                                  │
                                                  ▼
                                            sub=LEG2_DRIVE
                                            (前进到压线)
                                                  │
                                                  │ 压线
                                                  ▼
                                            _finish() ──► DONE
```

**几何路径（`_home_plan`）**：

| layout | leg1 (y1) | leg2 (y2) | 终点 |
|--------|-----------|-----------|------|
| 1 | href+180 | None | leg1 压线即 DONE |
| 2 | href+90 | href+180 | 两段都走 |
| 3 | href-90 | href+180 | 两段都走 |
| 4 | href+90 | None | 一段 |

---

## 9. Class 选择队列（`_remaining`）

```
       start()
       ─────► _remaining = match_order (例 [0,1,2])
                _filter_class = 7  (无偏好)
                _match_allow = None
                                │
       _set_pick_class()        │  (进入 HUNT/ALIGN 时调用)
       ─────────────────►       ▼
                          _match_allow = list(_remaining)
                          _filter_class = _remaining[0]  # 队首
                          _active_cls = None  (候选未锁定)

       见某 cls 连续 pick_class_frames 帧  ──► 插队首
       队首连续 pick_class_frames 帧没见    ──► 移队尾
       _lock_active_class(cls)              ──► _active_cls=cls, _filter_class=cls, _match_allow=None
       _remaining 已空                       ──► _fault("no target class left")
       scored_count++                        ──► 把该 cls 移到队尾（已推过的不重复）
       scored_count >= N                     ──► _want_home=True → 推完后回 HOME
```

**strict_target 模式**：`_remaining=[strict_cls]` 单类专用，跳过队列轮换。

---

## 10. 关键不变量与互斥

| 不变量 | 含义 |
|--------|------|
| `arbiter.owner == "MATCH"` 全程 | MatchRunner 独占电机；ISR force_brake 是唯一例外 |
| `_backoff_busy=True` 时 | 所有 `_write_*` 静默返回；不允许任何 phase 写电机 |
| `_yellow_hit=True` 时 | `isr_field_lock` 提前 return；主循环检测后必走 consume→BACKOFF |
| `_boundary_pending=True` ∧ `_boundary_need_cross=True` | ISR 不武装 BACKOFF，必须先压线再离线 |
| `phase ∈ (HUNT,ALIGN,PUSH)` 才允许场锁 | `field_lock_enabled` 谓词；LEAVE/HOME/BACKOFF 不触发黄线急停 |
| `consume_yellow_hit` 内 `if not _field_entered: ignore` | 防库外武装后压出库线误启动 BACKOFF |

---

## 11. 异常路径（FAULT 触发清单）

| 来源 | 信息 | 触发条件 |
|------|------|---------|
| `match.start()` | — | 已在 RUN |
| `_tick_hunt` | `cam timeout in HUNT` | `sensors.cam_timeout=True` |
| `_tick_hunt` | `HUNT timeout` | `pick_timeout_ms` 到 |
| `_tick_align` | `ALIGN total timeout` | `approach_cluster_timeout_ms` 到 |
| `_tick_push` | `PUSH timeout ... NOT scored` | `push_timeout_ms` 到（**未得分**） |
| `_tick_home` | `HOME timeout — gate not confirmed` | `home_timeout_ms` 到 |
| `_enter_hunt` | `HUNT failed (IMU not ready?)` | IMU 未标定 |
| `_skip_or_home` | `...; no target class left` | `_remaining` 空 |
| `_fault` 直接调用 | 用户自定义 | 任意 |

---

## 12. 时序示意（一轮成功跑完的预期路径）

```
t=0.0  IDLE         main 循环长亮
t=0.1  IDLE → RUN   C20 短按 → match.start() → phase=LEAVE
t=0.5  LEAVE        写 move_locked(hold_yaw); tcs.on_line → boundary wait enter
t=2.0  LEAVE → HUNT 见到目标 ∨ drive_timeout → _enter_hunt
t=2.5  HUNT/SPIN    队首 0/1/2 扫圈；见到 confirm_frames 帧 → TRACK
t=3.5  HUNT/TRACK   bearing_pid 控制；y2 渐增
t=5.0  HUNT → ALIGN push_yaw = push_hdg_ref + hdg_off[cls]
t=5.0  ALIGN/TURN   绕前方轴转到 push_yaw；yaw_ok + cx_ok 持续 → CLOSE
t=7.0  ALIGN/CLOSE  前近距离；y2 ≥ contact + cx_ok → PUSH
t=7.5  PUSH/DRIVE   前进 66%；推满 ~1.5s；cx 偏 → CORRECT
t=8.5  PUSH/DRIVE   重新居中；继续推
t=10.0 PUSH 压黄线   ISR 50Hz 检测 tcs.on_line → force_brake + _yellow_hit
t=10.0 主循环        consume_yellow_hit → _push_score_ready() True → _credit_score()
                    → _start_backoff() → phase=BACKOFF sub=RETREAT
t=10.0 BACKOFF       后退 ≥ backoff_retreat_min_ms；离线 → sub=SPIN
t=11.5 BACKOFF/SPIN  转 180°；≥ backoff_spin_deg → _finish_backoff
t=13.0 BACKOFF DONE  scored<N → _post_backoff="FWD" → _enter_hunt(forward=True)
t=13.0 HUNT/FWD      前进中心；超时或见目标 → SPIN/TRACK
...                  (下一轮)
t=∞   推够 N 个     scored_count >= N → _want_home=True → BACKOFF 后 POST=HOME
                    HOME → LEG1_TURN → LEG1_DRIVE → ... → DONE
```