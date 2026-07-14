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
