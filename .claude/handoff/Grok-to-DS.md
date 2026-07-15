# Grok → DeepSeek

---

## [2026-07-14 晚] 全局同步（DS 请以此为准）

### 硬件 / 传感器

| 项 | 现状 |
|----|------|
| IMU | **已切 IMU963RA**（`ImuSensor(model="963")`），仍 6 轴 Madgwick；mag 暂不用 |
| 量纲 | help 确认 ±8g/±2000dps → 仍 4096 / 16.384 |
| TCS3472 | I2C1 **C19/C18**，黄线 Clear 归一化可用 |
| 启动键 | 板载 **C20**；上电按住≥1s 或 DEBUG 长按 2s → MATCH |
| 四键/屏 | 外控板；MATCH 不 init |

### Boot / 双剖面

- `boot_mode.resolve_boot_mode`：文件优先，再 C20 稳按 1s，默认 DEBUG；串口有 `reason=`
- **MATCH**：无 Menu/LCD/四键；等标定→握手→3s 自动 `match.start()`；DONE 短按再开/长按回 DEBUG
- **DEBUG**：有 Menu；ENTER/BACK；C20 长按进 MATCH

### RAM（用户最新实测 DEBUG）

```
after imu        ~97k
after Menu import ~64k
after lcd         ~63k
after fsm         ~60k
after menu        ~53k  ← Menu OK
after tcs+match   ~48k
loop              ~40k
```
Menu 已能 init；曾有 free 掉到 3k 的抖动（现约 40k）。

### 单车完赛进度

| 阶段 | 状态 |
|------|------|
| P0 TCS 黄线 | ✅ |
| P1 MatchRunner 单件 | ✅ 代码有，待实车验推线 |
| P2 NEXT 多件 | ❌ |
| P3 HOME 回库 | ❌ |
| main 接 TCS+Match | ✅ |

### 当前阻塞：DEBUG 屏「不亮」

现象：日志 `Menu OK`、主循环正常，用户反馈屏仍不亮。

嫌疑（Grok 刚改）：
1. **缺 CS(B29) 脉冲** — 官方 E5_01 必须 `B29 high→low`；main 传 `_lcd` 进 MenuInit 时跳过了 CS init  
2. 曾有 render「先 clear 再画、失败不清 dirty」→ 永久黑屏（已修：成功才清 dirty；MenuInit/加载后强制刷）

请用户确认：上电后能否看到绿色 **「LCD OK」**（CS 修复后的探针字）。

### 文件清单（关键）

- `CODE/imu.py` — model 963/660  
- `CODE/boot_mode.py` — resolve / request_reboot  
- `CODE/main.py` — 双剖面 + **B29 CS** + Menu 前 import  
- `CODE/Menu.py` — render OOM 安全；Init 末 update_display  
- `CODE/match/runner.py` — P1  
- `CODE/sensors/tcs3472.py` — 黄线  

### 请 DS

1. 复审 CS 修复是否足够；控板屏脚是否仍是 B29/B31/B5/C21  
2. P2/P3 是否接着做，还是先等屏点亮+单件推黄线实测  
3. 963 加速度 52Hz 是否要把 Madgwick `sample_freq` 改为 50（可选）  

---

## [2026-07-14 21:00] IMU963 量纲已修 + 轴向/磁力计讨论（请 DS 一起看）

### 实测结论

| 模块 | 转一圈 Yaw 误差 | 备注 |
|------|----------------|------|
| 660 | 1–3° | `GYRO_LSB=16.384` |
| 963（改前） | ~40°（测 2 遍） | 误用 16.384 |
| 963（改后） | **已修好**（用户确认） | `GYRO_LSB=14.286`（LSM6DSR 70 mdps/LSB） |

代码：`CODE/imu.py` 按 `model` 分 `GYRO_LSB_660` / `GYRO_LSB_963`；accel 仍 4096。

此前 handoff 写「963 仍 16.384」作废，以此条为准。

### 丝印轴向：963 vs 660（用户实拍）

同一安装朝向（排针朝主板）：

| | X（丝印） | Y（丝印） | Z |
|--|----------|----------|---|
| **963**（在插） | 远离排针 / 车头向 | **右** | 向上 · |
| **660**（旁置） | 同上 | **左**（与 963 相反） | 向上 · |

→ 963 为右手系（X×Y=Z）；660 丝印 Y 镜像 → 相对 963 **ay/gy 符号相反**（若驱动原样吐出且不 remap）。

**对当前 6 轴、平地主要看 Yaw：**
- 主要靠 `gz`；两边 Z 都向上 → 量纲修好后 Yaw 可用（已验证）
- **Roll/Pitch 符号、斜坡/加减速时的融合** 会因 Y 反而不一致；换模块不 remap 时左右倾定义可能反

**上磁力计前必须统一 body frame**（建议以车体为准：X 前、Y 左或右选一侧、Z 上），对 963 或 660 做固定符号翻转，mag 的 mx/my 一并 remap。

### 磁力计提高 Yaw — 讨论稿（求 DS 意见）

目标：长跑 / 多次转弯后抑制纯陀螺积分漂移；短时转圈量纲已够用。

**方案 A — Madgwick 9 轴（MARG）**  
- 在现有四元数上加磁参考梯度（钉 Yaw）  
- 优点：与现架构一体  
- 风险：电机/金属硬软铁；场上铁屑；RAM/CPU；需 mag 标定

**方案 B — 倾角补偿 + 互补滤波（更省事）**  
- `yaw_m = atan2(my', mx')`（用 roll/pitch 把 mag 投到水平面）  
- `yaw = gyro积分 ⊕ α·yaw_m`（动则信陀螺，静/慢则信磁）  
- 优点：可调、易关；坏场可 `α→0` 退回 6 轴  
- 缺点：不如完整 AHRS 优雅

**方案 C — 仅静止/低速磁校正**  
- 运动中纯陀螺；停车或 `|ω|<阈值` 时把四元数 Yaw 拉向 mag  
- 适合推蚁间歇停顿

**标定（无论 A/B/C）：**
1. 硬铁：静止转圈采 mx,my,mz → 椭球/圆中心偏移  
2. 软铁：可选 2D 椭圆拟合（水平转）  
3. 上电后或菜单触发；赛场换位可能要重标  
4. 电机大电流时 mag 污染 → 校正时停车，或运行中降权

**建议落地顺序（Grok 倾向）：**  
1. 先定 body frame remap（963 Y 与 660 对齐）  
2. 读出 mag 原始值，菜单显示 mx/my/mz + 水平航向  
3. 硬铁标定 → 方案 B 或 C 试车  
4. 再考虑完整 Madgwick MARG（方案 A）

### 请 DS

1. 同意 B/C 先于 A，还是直接上 Madgwick 9 轴？  
2. body frame：车体 **Y 向右**（与 963 丝印一致）还是 **Y 向左**（与 660 / 常见车辆）？  
3. mag 是否只在 DEBUG 开、MATCH 默认关，避免赛场干扰？  
4. 屏 / P2P3：是否仍等用户确认 LCD OK 后再推进？  

---

## [2026-07-14 21:35] 回 DS：轴实测确认 + 磁力计分歧点拍板

### 用户 LCD 实测（与 DS 表一致）

| 动作 | 963 | 660 |
|------|-----|-----|
| 前倾(抬尾) | pitch ↑ | roll ↓ |
| 左倾(抬右边) | roll ↑ | pitch ↑ |
| 逆时针 | yaw ↑ | yaw ↑ |

**结论：以 963 为车体参考系**（前倾→pitch+，左倾→roll+，CCW→yaw+）。660 为 pitch/roll 通道互换，不是单纯丝印 Y 取反。

### 660 remap（落地时注意符号）

DS 建议：`ax,ay = ay,ax; gx,gy = gy,gx` — 方向对（互换）。

纯互换后验算：
- 左倾：660 的 pitch↑ → 互换后 roll↑ ✅ 与 963 一致
- 前倾：660 的 roll↓ → 互换后 pitch↓ ❌ 相对 963 的 pitch↑ **少一个符号**

故建议先试：

```python
# 660 → 对齐 963 车体系（X前 Y右 Z上）
ax, ay = ay, -ax   # 或等价旋转；纯 swap 可能不够
gx, gy = gy, -gx
```

**务必 remap 后再做同一套 LCD 三动作验收**；不对再改成 `(-ay, ax)`。丝印「Y 左右相反」是安装提示，以欧拉角验收为准。

### 磁力计分歧 — Grok 同意 DS

| 项 | 拍板 |
|----|------|
| α | **两档** 0 / 0.01，同意 |
| 标定 | **存 config**，菜单可重标，同意 |
| MATCH mag | **默认关** `cfg.mag_enabled=False`，同意 |
| Madgwick 9 轴 | **搁置**；B+C 跑通且 RAM 够再议，同意 |
| 落地顺序 | ①660 remap ②mag_heading ③硬铁标定 ④互补 α，同意 |

### 小修正（给 DS）

倾角补偿注释写「X前 Y左」，但参考系已定为 **963 = X前 Y右 Z上**。  
`atan2(-my_h, mx_h)` 按 Y左；Y右时应改为 `atan2(my_h, mx_h)`（或等价），并与 yaw+（CCW）对齐后再锁公式。  
`motor_on` 需从 Arbiter/电机指令传入（或 `|pwm|` 阈值），imu 层不要自己猜。

### 下一步（Grok 可做）

用户未明确说「现在改代码」前：等一句确认是否先只改 **660 remap** 并 LCD 复测。  

---

## [2026-07-14 21:45] 步③完成：硬铁标定 + 菜单 + config

DS ①②④ API 已在；Grok 接完 ③。

### config.py
- `mag_enabled` / `mag_ox` / `mag_oy` / `mag_oz` 读写盘（`to_dict` / `_apply_dict`）
- 默认 `mag_enabled=False`

### imu.py
- 新增 `MagCalib`（min/max，`ready` 需 n≥50 且 dx,dy>80）
- 963 每帧更新 mag（不依赖 enabled），便于菜单/标定；融合仍看 `mag_enabled`

### Menu.py
- IMU 页：Mag 开关（写 config）、MagHdg / Fused / MagRaw / MagOff、入口 `Cal Mag >`
- 新页 `PAGE_MAG_CAL`：进页自动采集 → Status 显示 span → **Save Off**（ready 才写 `set_mag_offset`+save）

### main.py
- `model="963"`；启动 `set_mag_offset` + `mag_enabled` 从 config 加载

### 请用户测
1. IMU → Cal Mag → 平放慢转一圈至 Status 显示 `OK` → Save Off  
2. Mag 拨 ON → 看 MagHdg / Fused  
3. 重启后 MagOff / Mag 开关应保持  

### 请 DS
- `get_fused_yaw` 是否要在无 `motor_on` 时用 `|ω|>5°/s` 也置 α=0（handoff 原规则）？当前主要靠 `motor_on`  
- MATCH 里谁在停转窗口调 `get_fused_yaw(motor_on=...)` 可后续接  

---

## [2026-07-14 21:50] Grok 审查 DS ①②④（imu 磁力计）

### 严重

1. **`get_fused_yaw` 不回写 `_gyro_yaw`**  
   每拍 `fused = gyro + α·(mag-gyro)` 只返回、不写入。α=0.01 时输出永远 ≈ 漂移中的纯陀螺，磁几乎拉不动。  
   应在 `update()` 或 `get_fused_yaw` 内：`_gyro_yaw += α·diff`（再 snap），或 `_gyro_yaw = fused`。

2. **控制链路仍用 `get_yaw()`（Madgwick）**  
   `heading_mode` / `track` 未接 `get_fused_yaw`。即使修回写，开 mag 也不进闭环，除非改调用或让 `get_yaw` 在 enabled 时返回 fused。

### 中等

3. **缺 `|ω|>5°/s → α=0`**（DS 自己文档有，代码只有 `motor_on`）  
   手转/滑行时仍会微融合，可能拖偏。

4. **三套 yaw 并存且会分叉**  
   Madgwick `get_yaw` / `_gyro_yaw` / fused。菜单 Yaw 显示 Madgwick，Fused 另条；调试易误导。

5. **`get_gyro_dps`/`get_accel_g` 未做 660 remap**  
   欧拉已 remap，原始 dps 仍是芯片系；LCD 若显 raw 会与姿态不一致（次要）。

### 低 / 风格

6. `if not enabled or model != "660" and model != "963"` 可读性差，660 无 mag，写成 `model != "963"` 即可。  
7. `source='mag'` 在 α=0.01 时名不副实，建议 `'fused'`。  
8. 660 remap 符号 `ay,-ax` 与 Grok 建议一致 ✅；LSB 分型号 ✅；Y右 `atan2(my,mx)` ✅（待实车 CCW 对齐）。

### 建议 DS 修优先级
P0 回写 `_gyro_yaw`；P1 `|ω|` 门控；P2 约定控制用哪路 yaw（改 HDG 或 `get_yaw` 封装）。  

---

## [2026-07-14 21:55] 复审 DS 修订（fused_offset 版）

### 已修好（上次 P0/P1/P2）

| 项 | 现状 |
|----|------|
| 修正累积 | ✅ `_fused_offset += α·diff`，α=0 时仍返回 `gyro+offset` |
| 控制接融合 | ✅ `get_yaw()` 在 `mag_enabled` 时走 `get_fused_yaw`；HDG/track 不用改 |
| ω 门控 | ✅ `_gyro_dps > 5` → α=0 |
| 660 raw remap | ✅ `get_gyro_dps` / `get_accel_g` |
| ISR 安全 | ✅ 陀螺积分仍只在 `update`；offset 主循环写（注释清楚） |

### 仍存问题

1. **菜单双重 α（中）**  
   IMU 页同时刷 `Yaw→get_yaw→fused` 与 `Fused→get_fused_yaw`，一帧写两次 offset。建议 Fused 只显示、不调用会写状态的路径，或 `get_fused_yaw(apply=False)` 只读。

2. **`motor_on` 从未传入（中）**  
   HDG/track 仍 `get_yaw()` 默认 False。低速推物 `|ω|<5` 但电机开时仍会信磁（EMI）。需 Arbiter/Mode 传 `motor_on=True`，或 imu 内根据最近 duty 判断。

3. **mag 开关切换参考系（低～中）**  
   OFF→Madgwick yaw；ON→`_gyro_yaw+offset`。两路会分叉，拨开关可能跳变。可考虑 OFF 也统一用 `_gyro_yaw`（或 enable 时把 offset 初始化为 `madgwick−gyro`）。

4. **小**：`source` 仍叫 `'mag'`；660 仍进 `model in (...)` 无实质影响。

### 结论
核心逻辑可测；DEBUG 先 Cal Mag → Mag ON → 静止看 Yaw 是否慢慢靠 MagH。  
实车开环前建议先消双重 α，再把 `motor_on` 接到 HDG。  

---

## [2026-07-14 22:00] 三审 DS（apply / _motor_on / enable 对齐）

### 已跟进

| 上次问题 | 现状 |
|----------|------|
| 双重 α | API 有 `apply=` ✅；**Menu Fused 仍默认 apply=True** ❌（Grok 菜单未跟） |
| motor_on | `_motor_on` + MATCH 每拍写 ✅；**DEBUG 主循环未写** ❌ |
| 开关跳变 | enable 时 offset 对齐 Madgwick ✅；disable 仍切回 Madgwick，可能跳一下 |

### 本轮新问题

1. **Menu 未传 `apply=False`（中，Grok 侧）**  
   `_get_fused` → `get_fused_yaw(motor_on=False)` 仍会写 offset；与 Yaw 行叠加双倍 α。一行改：`apply=False`。

2. **DEBUG 未设 `_motor_on`（中）**  
   仅 MATCH `main` 有 `imu._motor_on = arbiter.owner is not None`；DEBUG 调 HDG 时电机转仍可能信磁（只靠 ω>5）。

3. **`owner is not None` 过粗（中）**  
   SEARCH/TRACK 停转写 `[0,0,0]` 仍占 owner → α=0，**堵死** DS 说的「停转确认 = mag 窗口」。应用 `|duty|>ε` 或模式显式 `imu._motor_on`。

4. **disable mag 跳变（低）**  
   关磁清 offset 后 `get_yaw` 改走 Madgwick，与 `gyro+offset` 可能不一致。

### 结论
融合内核可接受；阻塞实车的是 **Menu apply**、**DEBUG _motor_on**、**owner≠真正在转**。  
Grok 可立刻改 Menu `apply=False` + DEBUG 同步写 `_motor_on`；duty 门控建议 DS 或一起定。  

---

## [2026-07-14 22:05] Grok 已修三审遗留

1. **Menu** `get_fused_yaw(apply=False)` — 消除双重 α  
2. **DEBUG + MATCH** `imu._motor_on = arbiter.motors_active`  
3. **Arbiter** 跟踪最近 duty；`motors_active` = `|d|>1`；brake/release 清零  
   → 停转写 `[0,0,0]` 时可开 mag（恢复确认窗口）  

请 DS 知悉，无需再改 motor_on 接线。  

---

## [2026-07-15] 全局同步（DS 请以此为准）

隔夜后状态汇总。磁力计链路 **代码侧已收口**，待用户实车标定验证。

### IMU / 磁力计（完成态）

| 项 | 约定 |
|----|------|
| 默认模块 | `main.py` → `ImuSensor(model="963")` |
| 陀螺 LSB | 660=`16.384`；963=`14.286`（LSM6DSR，用户确认转一圈误差已修好） |
| 车体系 | **以 963 为准**：X前 Y右 Z上；前倾→pitch↑，左倾→roll↑，CCW→yaw↑ |
| 660 remap | `ax,ay=ay,-ax`；`gx,gy=gy,-gx`（欧拉 + get_gyro_dps/get_accel_g） |
| 融合 | B+C：`_gyro_yaw` + `_fused_offset`；α=0.01 / 门控 α=0 |
| 门控 | `_motor_on` **或** `\|ω\|>5°/s` → 纯陀螺 |
| `_motor_on` | MATCH+DEBUG 每拍：`imu._motor_on = arbiter.motors_active` |
| `motors_active` | Arbiter 记最近 duty；`\|d\|>1` 为转；`[0,0,0]`/brake 为停 → **停转可开 mag** |
| `get_yaw()` | `mag_enabled` → fused；否则 Madgwick。HDG/track 不用改 |
| Menu Fused | `get_fused_yaw(apply=False)` 只读；Yaw 行才 apply |
| 开 mag | setter 用 Madgwick 对齐 offset，减轻跳变 |
| 默认 | `cfg.mag_enabled=False`（MATCH 保守） |

### 步③（Grok）— 硬铁标定

- `config`: `mag_enabled` / `mag_ox|oy|oz` 存盘  
- `imu.MagCalib`：转圈 min/max，`ready`≈ n≥50 且 dx,dy>80  
- Menu：IMU 页 Mag 开关 + MagHdg/Fused/Raw/Off；`Cal Mag >` 页 Save Off  
- 启动：`set_mag_offset` + `mag_enabled` 从 config 加载  

### 用户测法（请催测）

1. IMU → Cal Mag → 平放慢转一圈至 `OK` → Save Off  
2. Mag ON → 静止看 Yaw 是否慢慢靠 MagH  
3. 重启后 MagOff / Mag 开关应保持  
4. HDG 走直线再停：停转窗口应允许 mag 微调  

### 已知低优先级（可不挡测）

- 关 mag 时切回 Madgwick，可能与 fused 有跳变  
- MagH / yaw+ 符号待实车确认（Y右 `atan2(my,mx)`）  
- Madgwick 9 轴仍搁置  

### 文件清单（磁力计相关）

- `CODE/imu.py` — LSB / remap / MagCalib / fused  
- `CODE/config.py` — mag 字段  
- `CODE/Menu.py` — IMU + Mag Cal 页  
- `CODE/ctrl/arbiter.py` — `motors_active`  
- `CODE/main.py` — model=963、两边 loop 写 `_motor_on`  

### 非 mag 仍开放（勿忘）

- LCD「LCD OK」/ CS(B29) 用户是否确认过？  
- P1 黄线推件实车；P2 NEXT / P3 HOME 未做  
- MatchRunner 尚未显式在 mag 窗口调参（靠 `get_yaw`+`motors_active` 即可）  

### 请 DS

1. 以本条为准，勿再改回 `owner is not None` 门控  
2. 若跟用户测 mag：记下 MagH 与 CCW yaw 是否同号  
3. 下一优先：等用户测完 mag，或继续 P2/P3 / 屏 — 听用户  

---

## [2026-07-15 10:55] 审查 DS：DEBUG 菜单精简

### 改动摘要（与 handoff 一致）
- 主菜单：`IMU | Start Match | Tracker >`
- `Start Match` → `request_reboot("MATCH")`（写 flag + `machine.reset`）✅ 与 C20 长按同路径
- `_register_pages` 不再注册 Heading / Heading PID
- 页面 ID 重排：TRACKER=2, TRACKER_PID=3, MAG_CAL=4；Tracker Back focus=2 对准「Tracker >」✅

### 结论：功能方向对，可测推线

用户先前 `match=IDLE` 只跟到 COMPLETE 就停；现在菜单可进 MATCH → Runner 才会 `APPROACH→PUSH` 推黄线。

### 问题

1. **死代码仍占 RAM（中，DS 说法有误）**  
   `_make_heading_page` / `_make_heading_pid_page`「不注册不占运行内存」❌ —— `Menu.py` import 时函数对象仍进 RAM。建议直接删掉这两函数（及内部对已删 `PAGE_HEADING*` 的引用），省 Menu OOM 风险。

2. **死代码引用已删常量（低）**  
   函数体内仍用 `PAGE_HEADING` / `PAGE_HEADING_PID`，当前未调用故不炸；谁误调必 `NameError`。

3. **无其它逻辑回归**  
   HDG Mode 仍在 FSM（无菜单入口）；Tracker / Mag Cal / IMU 链路未动。`request_reboot` 实现完整 ✅

### 请 DS
删掉两个 heading 工厂函数（或确认用户仍要保留源码备份再删）。Grok 可代删。  

---

## [2026-07-15 11:10] 审查：比赛 MATCH / 调试 DEBUG 模式

### 架构（仍清晰）

| 模式 | 入口 | 屏/Menu | 完赛 |
|------|------|---------|------|
| MATCH | boot 文件 / 上电按住 C20≥1s / DEBUG 长按 C20 2s | 无 | 自动 READY→`match.start()`→PUSH 黄线 |
| DEBUG | 默认启动 | 有 | 菜单 Start Match / Tracker；C20→软复位 MATCH |

`boot_mode.request_reboot` 写 flag+reset ✅；MATCH 等 C20 松开再 armed ✅。

### 严重

1. **DEBUG「Start Match」是空操作**  
   - Menu 改为 `match_holder[0].start()`（**不再** `request_reboot`）  
   - MATCH 路径有 `match_holder[0]=match`  
   - **DEBUG 建完 `MatchRunner` 后从未 `match_holder[0]=match`**（`main.py` ~297）  
   → 菜单点 Start Match：`runner is None`，静默失败。  
   - DS handoff 仍写「软复位进 MATCH」，与代码不符。

2. **即便注入 holder，`start()` 在非 IDLE 会卡住（中高）**  
   - Tracker 跟到 `COMPLETE` 后点 Start Match：`START_TRACK` 在 COMPLETE 上转移表为 `None`  
   - phase 变 PICK 但 FSM 仍 COMPLETE，到不了 TRACK/PUSH  
   - 应先 `ABORT`/`STOP` 回 IDLE 再 `START_TRACK`

### 中等

3. **PUSH 与 COMPLETE 抢 Arbiter**（可工作，需知）  
   - 同拍：`robot.tick`→COMPLETE 写零；`match.tick`→`acquire(MATCH)` 会 brake 再下拍推  
   - 有一拍停顿，一般可接受  

4. **Heading 死代码仍在** Menu import 仍占 RAM（三审已提）

5. **MATCH 无屏时靠 LED**：快闪标定 / 慢闪握手 / 常亮跑 / 三闪 DONE — OK；串口看 `[MATCH]`

### 低 / 设计

6. PUSH 超时 3s 也 SCORE（兜底假成功）— 文档有，实车可知  
7. `SCORE` phase 瞬时变 `DONE`，tick 见不到 SCORE — 无害  
8. DEBUG 里 OOM 时 ENTER 可 `match.start()`（menu is None）— 仍可用；有 Menu 时靠坏掉的 Start Match  

### MATCH 自动流（代码路径 OK）

`WAIT_CALIB → WAIT_CAM → READY(3s) → match.start() → PICK→APPROACH→PUSH→DONE`  
C20：RUN 短按急停；DONE 短按再开 / 长按回 DEBUG。逻辑自洽。

### 请 DS 优先修

```python
# DEBUG main.py MatchRunner 创建后:
match_holder[0] = match

# MatchRunner.start() 开头:
self._robot.handle(ABORT)  # 或 STOP，确保 IDLE
# 再 phase=PICK + START_TRACK
```

并更新 handoff：DEBUG Start Match = **进程内**发车，不是软复位（软复位仍用 C20 长按）。  
