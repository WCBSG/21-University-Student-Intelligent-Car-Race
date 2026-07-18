# DeepSeek → Grok

> 激进重构方案：MCU 代码架构重设计
> 源文件：`C:\ProgramFiles\Claude\.claude\plans\valiant-strolling-cherny.md`（完整方案）
> 日期：2026-07-19

---

## 一、重构动机

当前代码（commit `6d3ca09`）架构债严重，且 07-19 实测 **仅 1/3 进球**，耗时 ~294s 仍未完赛：

| 架构问题 | 严重度 | 影响 |
|----------|--------|------|
| `MatchRunner(MatchIsr, MatchHunt)` 多重继承 | 🔴 | 3 文件共享 60+ 实例变量，零封装 |
| ISR-主循环通过 10+ 可变标志通信 | 🔴 | `_yellow_hit`/`_def_score`/`_boundary_armed`… 跨上下文读写，无法静态分析 |
| `if phase == "HUNT"` 字符串派发链 | 🟠 | 25 子状态全字符串，拼写错误静默吞没 |
| God Class ~600 行 | 🟠 | 状态机/电机/计分/导航全耦合 |
| `sensors.get("target")` 裸字典 32 处 | 🟠 | 无类型安全 |
| `_backoff_busy` 拦截所有 `_write_*` | 🔴 | 隐式优先级，新 phase 忘了加守卫就崩 |

**运行时问题**（07-19 log）：dt 钳位 122 次、GC 触发 455 次、PUSH skew 死锁 15s、yaw 漂移 ±180°。

---

## 二、推荐方案：显式 FSM + 事件总线 + 组合优于继承

### 新文件结构

```
CODE/
├── main.py           # ~150 行（原 428）
├── fsm.py            # ★新★ State + Machine 框架（~60 行）
├── brain.py          # ★新★ 状态定义 + 策略（~500 行）
├── world.py          # ★新★ SensorReading + MotorCmd + EventRing + BoundaryLock（~300 行）
├── config.py         # 改造：_KEY_MAP 替代 400 行 if-elif
├── imu.py / motion.py / camera.py / tcs3472.py / log.py  # 不动
├── config.json       # 不动
```

**删除**：`match.py`(564行) + `match_hunt.py`(559行) + `match_isr.py`(200行) = **-1320 行**

### 核心 5 个抽象

#### 1. `fsm.py` — State 类 + Machine 派发器
```python
class State:
    name = ""
    def enter(self, ctx): pass
    def tick(self, ctx, dt, world) -> (next_state | None, [MotorCmd]): pass
    def exit(self, ctx): pass

class Machine:
    def tick(self, ctx, dt, world):
        next_state, cmds = self._state.tick(...)
        if next_state: self._state.exit(); self._state = next_state; self._state.enter()
        return cmds
```
**所有 State 实例是模块级单例**（import 时创建一次，`tick()` 中零对象分配）。

#### 2. `world.py` — EventRing（替代 10+ 可变标志）
```python
# ISR 安全 SPSC 环形缓冲区 — ISR 端零分配
class EventRing:
    def __init__(self, cap=8): self._buf = [(0,0)]*cap  # 预分配
    def push(self, typ, data=0): ...   # ISR 调
    def pop(self): ...                 # 主循环调
```
ISR → `push(EVT_YELLOW)` → 主循环 `drain()` → 处理计分/BACKOFF。

#### 3. `world.py` — SensorReading（替代裸字典）
```python
class SensorReading:
    __slots__ = ('new_frame', 'has_target', '_target', 'cam_timeout', 'tcs_on_line')
    @property
    def cx(self): return float(self._target[6]) if self._target else 50.0
    @property
    def y2(self): return float(self._target[9]) if self._target else 0.0
    @property
    def cls_id(self): return int(self._target[0]) if self._target else -1
```
替代所有 32 处 `sensors.get("target")` / `float(target[6])`。

#### 4. `world.py` — BoundaryLock（替代 6 态边界标志）
```python
class BoundaryLock:
    """场锁 FSM: DISABLED → PENDING → ARMED → HIT
       ISR 调 isr_tick() → push 事件到 EventRing；主循环 drain 后处理"""
    def arm_when_clear(self): ...   # phase 进入时调用
    def isr_tick(self): ...         # 50Hz — 零分配、零打印、零电机
```
替代 `_boundary_armed`/`_boundary_pending`/`_boundary_need_cross`/`_boundary_saw_line`/`_field_entered`/`_boundary_arm_ms`。

#### 5. `brain.py` — 20 个 State 类
```
State 层次:
  Idle, Leave
  Hunt → HuntSpin, HuntFwd, HuntTrack
  Align → AlignTurn, AlignClose
  Push  → PushDrive, PushCorrect
  Backoff → BackoffRetreat, BackoffSpin
  Home  → HomeLeaveLine, HomeLeg1Turn, HomeLeg1Drive, HomeBackoff, HomeBackoffTurn, HomeLeg2Drive
  Done, Fault
```

每个 State 约 20-50 行，`enter()` / `tick()` / `exit()` 三方法。转移通过返回 State 单例实现。

---

## 三、主循环简化

```python
# main.py 精简后的主循环
while True:
    sensors = SensorReading.build(camera, tcs, cfg, brain.allow, brain.filter_cls)

    # 处理 ISR 事件（替代 flush_deferred + consume_yellow_hit）
    for evt in events.drain():
        if evt == EVT_YELLOW:   brain.on_yellow_hit()
        elif evt == EVT_ARMED:  log("boundary armed")
        elif evt == EVT_ENTER:  log("entered field")

    # FSM tick → 电机命令
    cmds = fsm.tick(brain, dt, sensors)
    for cmd in cmds:
        arbiter.execute(brain.OWNER, cmd)

    sleep_ms(5)
```

---

## 四、config.py 改造

400 行 `if k == "航向P": ... elif` → 声明式映射表：

```python
_KEY_MAP = {
    "航向P":     ("heading", "kp", float),
    "行驶占空比": ("drive_duty", None, float),
    "搜索顺序":   ("match_order", None, lambda v: [int(x) for x in v]),
    # ... 70+ key
}
def _apply_dict(self, d):
    for k, v in d.items():
        entry = _KEY_MAP.get(k)
        if entry:
            sub, attr, fn = entry
            setattr(getattr(self, sub) if sub else self, attr, fn(v))
```

---

## 五、迁移策略（4 阶段）

### 阶段 1：基础设施（不破坏现有代码）
1. 新建 `fsm.py`、`world.py`（EventRing/SensorReading/MotorCmd）
2. 在 MatchRunner 中**并行**加 EventRing — ISR 同时写旧标志和新 ring
3. 重构 `config.py` — 加 `_KEY_MAP`，旧 `_set_one` 保留 fallback

### 阶段 2：状态机迁移（逐个 phase）
4. 新建 `brain.py` + 所有 State 单例
5. `main.py` 加 `USE_NEW_FSM` 开关，并行 A/B 对比 NAV log
6. 逐个 phase 切：IDLE/LEAVE → HUNT → ALIGN/PUSH → BACKOFF/HOME

### 阶段 3：清理
7. 删除 `match.py` / `match_hunt.py` / `match_isr.py`
8. 删除旧分支，更新 `build_flash.py`

### 阶段 4：运行时修复
9. `PushCorrect` 加 4s timeout → 超时回 HuntSpin
10. `HuntSpin` 加每类 8s 上限 + `vision_lost` 前置条件
11. `BackoffSpin.exit()` 重置 IMU 零位 + 回场心前进 1s
12. 急转禁磁融合（`gyro_dps > spin_dps`）

---

## 六、备选方案（已评估不选）

| 方案 | 为什么不选 |
|------|-----------|
| 行为树 | 比赛流程本质线性，BT 过度设计；MicroPython 无现成库 |
| 协程/生成器 | MicroPython 栈限制；ISR 无法 yield；调试困难 |
| 微调当前架构 | 不解决根本问题 — 用户要求"激进" |

---

## 七、期望效果

- 多重继承消除 → 每个模块可独立理解和测试
- 事件总线替代可变标志 → ISR/主循环通信显式化、可追踪
- 显式 FSM → 状态转移一目了然，不再有字符串拼写错误
- 类型化传感器 → IDE 补全、拼写检查、零 `None.get()` 崩溃
- config 映射表 → 新增参数改 1 行（vs 当前 3 处）
- 净减 ~755 行代码

**不开工，等 Grok 审阅后再决定是否执行。**
