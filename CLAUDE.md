# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## 项目概述

基于 **NXP RT1021 (i.MX RT1021)** + 逐飞科技核心板的 MicroPython 固件开发。裸机运行——无 OS、无 CPython、无标准库（仅 MicroPython builtins）。`.py` 文件通过 USB 上传到核心板 `/flash` 目录，由板载 MicroPython 解释器执行。

**工具链**：Thonny IDE 烧录/REPL；VSCode 编辑 + `.stubs3.1.0/` 类型存根。存根对应 **固件 v3.1.0**。

**辅助 MCU**：OpenART Plus（NXP RT1064）作为摄像头协处理器，通过 UART 与 RT1021 通信，运行独立的 MicroPython 固件。

---

## ⚠️ 文件同步规则（必读）

本项目代码需同步到两块硬件上运行，修改文件后**必须提醒用户手动同步**：

### 同步目标 1：OpenART Plus SD 卡

| 源目录 | 目标 | 方式 |
|--------|------|------|
| `CameraCode/` 下**除 `【例程】OpenART Plus例程/` 外**的所有 `.py`/`.md`/`.bat` 文件 | OpenART SD 卡根目录 | 用户手动复制（SD 卡读卡器或 OpenART USB 大容量模式） |

需要同步的文件包括：`camera_test.py`、`main.py`（即 `camera_test.py` 的副本，上电自动运行）、`(MCU)test_camera.py`、`摄像头测试流程.md`、`摄像头文件.bat`

### 同步目标 2：RT1021 核心板 /flash

| 源目录 | 目标 | 方式 |
|--------|------|------|
| `CODE/` 下**除 `[例程]Rt1021例程/` 外**的所有 `.py` 文件 | RT1021 `/flash` 目录 | 用户通过 Thonny IDE 手动上传 |

需要同步的文件包括：`main.py`、`Motor.py`、`Menu.py`、`imu.py`、`HeadingController.py`、`CameraReceiver.py`、`ObjectTracker.py`、`config.py`

### 同步规则

- **每次修改 CODE/ 或 CameraCode/ 下的文件后，必须主动提醒用户需要同步到哪块硬件**。
- `【例程】` 目录是官方示例，**只读**，不同步、不修改。

---

## 仓库结构

```
.stubs3.1.0/            ← VSCode/Pylance 类型存根
  machine/              ← Pin, ADC, PWM, UART, SPI, I2C, SoftI2C
  seekfree/             ← IMU660RX, IMU963RX, MOTOR_CONTROLLER, KEY_HANDLER, DL1X, TSL1401, WIFI_SPI
  smartcar/             ← ticker, encoder, ADC_Group
  display/              ← LCD_Drv, LCD
  time/ os/
.vscode/                ← Pylance stubPath + extraPaths 配置
CODE/                   ← RT1021 端用户程序（需同步到 /flash）
  main.py               ← 入口：硬件初始化 → ticker → 主循环
  Motor.py              ← MotionControl: 3×DRV8870DDAR 全向轮驱动
  Menu.py               ← IPS200 环形菜单系统
  imu.py                ← IMU660RX + Madgwick AHRS 姿态解算
  HeadingController.py  ← 航向角 PID 闭环 (HeadingPID + HeadingController)
  CameraReceiver.py     ← OpenART 检测结果 UART 接收器
  ObjectTracker.py      ← 视觉伺服状态机（搜索→追踪→停车）
  config.py             ← /flash/config.json 持久化配置
  [例程]Rt1021例程/      ← 逐飞官方 RT1021 示例 — 只读
CameraCode/             ← OpenART Plus 端程序（需同步到 SD 卡）
  camera_test.py        ← OpenART 端通信测试（4 种测试模式）
  main.py               ← camera_test.py 副本，上电自动运行 run(test_number=3)
  (MCU)test_camera.py   ← RT1021 端配对通信测试
  摄像头测试流程.md       ← 分步测试指南
  摄像头文件.bat          ← OpenART SD 卡文件管理批处理
  【例程】OpenART Plus例程/ ← OpenART Plus 官方示例 — 只读
Model/                  ← PC 端 YOLOv3-tiny 训练流程 (Python 3.10, TF2.x)
  yolo3_smartcar/       ← 训练脚本、模型、TFLite 输出
  数据/                  ← VOC 格式训练数据
RT1021-MicroPython 固件接口说明.pdf  ← 官方固件 API 文档
```

---

## 关键约束

- **`【例程】` 是只读的**。当存根与示例模式冲突时，修**存根**，不改示例。
- **只 import 存在的东西**。无 `argparse`、`logging`、`unittest`、`typing`(运行时)、`pathlib`、`threading` 等。仅 MicroPython builtins。
- **文件 I/O 通过 `io.open()` 和 `os`**。Flash 文件系统位于 `/flash`。使用绝对路径。不要在循环中反复 open/close 文件——Flash 写入次数有限。
- **引脚名是字符串**：`'C4'`、`'B27'`、`'D9'`，不是整数。
- **UART 缓冲区 63-64 字节**。OpenART 和 RT1021 的 MicroPython 固件均有约 64 字节的内部 UART 环形缓冲区。`uart.write()` 超过 63 字节会被静默截断；`uart.any()` 返回值不超过 63。解决方案：分块写入 ≤60 字节 + 2ms 块间延迟；接收时增量读取（不要 `while uart.any() < N` 等 N>63）。参见 `CameraCode/camera_test.py` → `write_chunked()` 和 `recv_frame()`。460800 baud ≈ 46 KB/s → 60 字节 ≈ 1.3ms 线缆传输时间。

---

## MicroPython 特有模式

### 全局运行时常量
`BOARD_TYPE` 和 `BOARD_VERSION` 无需 import 即可使用——它们是 `str` 值，如 `'RT1021_144P_BTB'`、`'RT1021_144P_2P54'`、`'RT1021_100P_2P54'`。

### 传感器数据缓冲区自动刷新
`imu.get()`、`key.get()`、`encoder.get()` 返回**链接缓冲区**——调用一次 `.get()`，保留返回的 list 引用，其内容在每次 ticker 驱动的 `.capture()` 时自动刷新。**不要每 tick 调用 `.get()`**。

### ticker（smartcar 模块）
- `ticker(id)` — PIT 通道 [0, 3]。通过 `start(ms)` 设置 tick 周期（最小 5ms）。
- `capture_list(*modules)` — 注册传感器（ADC_Group, encoder, KEY_HANDLER, IMU*, DL1X, TSL1401）以实现每个 tick 自动 `.capture()`。
- `callback(handler)` — handler 接收 ticker 实例作为唯一参数。

### KEY_HANDLER
- `get()` 返回 `[k1, k2, k3, k4]` — 0=无按下, 1=短按, 2=长按。
- `clear(index)` — index **从 1 开始**（1–4）。`clear()` 无参数时清除全部。

### IMU 传感器
- **IMU660RX**（6 轴）：`get()` → `[ax, ay, az, gx, gy, gz]`。可通过 `quar_rate` 参数获取硬件四元数（需 INT2→D19，与 `ticker.capture_list` 不兼容）。
- **IMU963RX**（9 轴）：`get()` → `[ax, ay, az, gx, gy, gz, mx, my, mz]`。`quar_rate` 仅支持 `RATE_DISABLE`——所有姿态估计需软件完成。
- 两者在硬件未连接时抛出 `ValueError: Module init fault.`——务必用 try/except 包裹 init。
- 原始值→物理量：加速度计 ±8g → 4096 LSB/g；陀螺仪 ±2000dps → 16.384 LSB/(deg/s)；磁力计 ≈ 16 LSB/uT (BMM150)。

### MOTOR_CONTROLLER
- 占空比范围 `[-10000, 10000]`。引脚对常量：`PWM_C30_DIR_C31` (PWM+DIR)，`PWM_C30_PWM_C31` (PWM+PWM) 等。
- `duty()` 无参数返回当前值；带参数设置并返回。

### IPS200 显示屏
- `LCD_Drv.LCD200_TYPE` + `LCD(spi_index, baudrate, dc_pin, rst_pin, lcd_type)`。
- 原始绘制：`lcd.str24(x, y, text, color)`、`lcd.clear(color)`、`lcd.mode(n)`。
- `IPS200PRO` 是 GUI 控件库（页面、标签、表格）；`LCD` 是原始绘制层。仅字体大小 16/20/24 支持中文。

---

## OpenART Plus — 摄像头协处理器通信

OpenART Plus 是独立的 NXP RT1064 MCU，带摄像头 + NPU，运行独立的 MicroPython 固件。通过 **UART (LPUART6: TX=D20 → OpenART RX, RX=D21 ← OpenART TX, 460800 baud)** 与 RT1021 连接，发送目标检测结果（成帧二进制数据包）。

### 协议帧

```
[0xAA] [CMD:1B] [LEN:1B] [DATA:N bytes] [CRC:1B]
```

CMD：
- `0x01` — MCU→OpenART: Connection Request（新增，空 payload）
- `0x02` — OpenART→MCU: Self-Test Result（新增，payload="200"/"400" ASCII）
- `0x03` — MCU→OpenART: Start Detection（新增，空 payload）
- `0x10` — OpenART→MCU: Detection Results
- `0x20` — UTF-8 文本
- `0xF0` — 回环测试

### 连接握手（TCP 式 3 步）

```
MCU                                    OpenART
 |                                        |
 |--- CMD 0x01 (connection req) --------->|  自检完成 → 阻塞等待
 |                                        |
 |<-- CMD 0x02 (self-test result) --------|  "200"=OK, "400"=FAIL
 |                                        |
 |--- CMD 0x03 (start detection) -------->|  开始检测循环
 |                                        |
 |<-- CMD 0x10 (detection frame) ---------|  正常数据流
```

MCU 端重试策略：发 0x01 → 等 100ms → 超时重试，最多 100 次（10 秒）。收到 400 直接放弃。期间 LCD 显示 "Wait Camera Connect... N/100" + 动态点。

### 检测结果负载格式 (CMD 0x10)

```
[num:1B] [cls_score:1B x:1B y:1B w:1B h:1B] × N
```
- `cls_score` = (cls:3bit << 5) | score:5bit。cls: 0-7 (7=任意)，score: 0-31（原始刻度）
- x, y = 左上角坐标 (0-255 → ÷2.55 = %)
- w, h = 宽高 (0-255)
- MCU 端解析时预计算 cx, cy, area, y2

### 转义机制

0xAA→0xBB 0x00, 0xBB→0xBB 0x01。对负载应用转义后组帧，防止出现伪同步头。

### OpenART SD 卡文件

Model + labels 放在 SD 卡上：
- `yolo3_iou_smartcar_final_with_post_processing.tflite` — 检测模型
- `cmm_cfg.csv`、`cmm_load.py` — SD 卡启动文件
- `labels_animal_fruits.txt` — 类别名（分类模型用）

### OpenART MicroPython 差异

- UART 导入可能是 `from machine import UART` 或 `from pyb import UART` — `camera_test.py` 两者都试
- `import tf` 加载 TensorFlow Lite 运行时（**仅 OpenART 可用**，RT1021 上不存在）
- `sensor` 模块控制摄像头（reset, snapshot, set_pixformat 等）
- `image` 模块用于绘图（draw_rectangle, draw_string）


### 测试流程

详见 `CameraCode/摄像头测试流程.md`。快速步骤：
1. OpenART: `import camera_test; camera_test.run(test_number=1)` — UART 回环自检（短接 TX-RX）
2. OpenART: `camera_test.run(test_number=4)` + RT1021: `import test_camera; test_camera.start()` — MCU 联调通信
3. OpenART: `camera_test.run(test_number=3)` + RT1021: `test_camera.start()` — 全功能检测+通信
4. RT1021 追踪：上传 CODE/ 文件 → `import main` 或上电自动运行 → 菜单 "Tracker >" → Start Track

---

## PC 端模型训练流程

YOLOv3-tiny，TensorFlow 2.x，Python 3.10。**非 MicroPython**——在 PC 上运行，产出 `.tflite` 文件部署到 OpenART SD 卡。

### 数据
- `Model/数据/` — VOC 格式：`JPEGImages/*.jpg` + `Annotations/*.xml`
- 类别：netball (0), sandbag (1), bear (2)（注意：camera_test.py 中 class_names 顺序为 `['netball', 'sandbag', 'bear']`）

### 训练流程（均通过 `Model/Model.bat`）
1. `voc_convertor.py` — VOC XML → train_data.txt
2. `kmeans.py` — 生成 anchor boxes → yolo3_anchors.txt
3. `train.py` — 训练 ResNet-based YOLOv3-tiny，输出 `.tflite`
4. `evaluate.py` — mAP 评估
5. `detect.py` — 单图推理测试（`-model X.tflite -image test.jpg`）

### 自动标注
`Model/auto_label.bat` 运行 `auto_label.py` — 用已训练模型预标注新图片（半自动数据集扩展）。输出 VOC XML 到 `Annotations/`。

### 标注查看器
`Model/view.bat` 运行 `view_annotations.py` — 读取 VOC XML + JPEG 图片，生成 HTML 画廊供浏览标注。自动打开浏览器。

### 关键文件
| 文件 | 用途 |
|------|------|
| `config.cfg` | voc_folder 路径、类别名、训练超参数 |
| `train_data.txt` | 生成的标注列表 (image_path x1,y1,x2,y2,class_id ...) |
| `yolo3_anchors.txt` | K-means 计算的 anchor boxes |
| `yolo3_iou_smartcar_final.tflite` | 训练好的模型（无后处理） |
| `yolo3_iou_smartcar_final_with_post_processing.tflite` | 含 NMS 后处理的模型 → **部署到 OpenART SD 卡** |
| `tflite_add_post_processing.py` | 向普通 TFLite 模型添加 NMS/检测后处理 |

---

## CODE/ 程序架构

多文件应用，加载到 `/flash`，由板载 MicroPython 解释器执行。

### 模块一览

| 模块 | 职责 |
|------|------|
| `MotionControl` (`Motor.py`) | DRV8870DDAR 双 PWM 半桥 × 3 全向轮。速度范围 [-100, 100]%。`stop()`=滑行(LOW/LOW), `brake()`=刹车(HIGH/HIGH)。PWM 写入间隔 `sleep_us(76)` 作为 DRV8870 死区时间。`move(speed, angle)`=全向逆运动学解算。 |
| `ImuSensor` (`imu.py`) | IMU660RX 封装 + Madgwick AHRS 融合：启动陀螺仪零偏标定 → 在线 EMA 零偏跟踪 → 四元数姿态估计。暴露 `get_yaw()/pitch()/roll()`、`recalibrate()`、`is_calibrated`、`bias_dps`。 |
| `HeadingPID` + `HeadingController` (`HeadingController.py`) | 增量式 PID + 航向闭环控制器。两种模式：`straight`（锁定当前航向+前进）、`lock`（原地旋转到目标角度）。PID 带死区 + 反算抗饱和。**`HeadingPID` 被 `ObjectTracker` 复用于视觉伺服 bearing PI**。 |
| `CameraReceiver` (`CameraReceiver.py`) | 非阻塞 UART 帧接收器。封装 OpenART 协议栈（CRC、转义、帧解析）。`update()` 在收到新 CMD 0x10 检测帧时返回 `True`。每 1s 发送心跳 (CMD 0x0F)。**接受外部传入的 UART 对象，不在内部创建**。 |
| `ObjectTracker` (`ObjectTracker.py`) | 4 状态视觉伺服状态机：`IDLE → SEARCHING → TRACKING → COMPLETE`。SEARCHING：匀速旋转搜索目标。TRACKING：bearing PI（复用 `HeadingPID`）+ 恒定低速前进。COMPLETE：bbox 底边 ≥ 画面 95% 时触发滑行停车。丢失 4 帧后反转 30° 回头扫描。**不估计距离——每帧独立决策**。 |
| `Menu` (`Menu.py`) | 环形菜单控制器。编辑模式下 UP/DOWN 在 min/max 边界循环。页面 0~6：Main/IMU/About/Heading/HeadingPID/Tracker/TrackerPID。 |
| `config` (`config.py`) | JSON 配置持久化：原子写入（tmp + rename），首次启动自动创建默认值。 |
| `main.py` (模块级) | 硬件初始化 → 创建 UART(5) → CameraReceiver + ObjectTracker → 构建菜单 → 启动 ticker(10ms) → 主循环：BACK 安全优先 → camera 轮询 → tracker/hdg 互斥 → 显示刷新。 |

### 配置键（持久化到 `/flash/config.json`）

| 键 | 默认值 | 使用者 |
|-----|--------|--------|
| `target_speed` | 50.0 | 直行模式前进速度 |
| `heading_kp/ki/kd` | 2.0, 0.0, 0.0 | 航向 PID 增益 |
| `heading_max_correction` | 50.0 | PID 输出限幅 — 最大旋转占空比 |
| `heading_deadband` | 1.0 | 航向死区 (deg) |
| `trk_bearing_kp/ki/kd` | 1.5, 0.05, 0.0 | 视觉伺服 bearing PI 增益 |
| `trk_bearing_max` | 60.0 | 最大旋转修正 (%占空比) |
| `trk_bearing_db` | 0.02 | bearing 死区（归一化值） |
| `trk_approach_speed` | 15.0 | 追踪前进速度 (%占空比) |
| `trk_search_speed` | 15.0 | 搜索旋转速度 (%占空比) |
| `trk_target_class` | 255 | 目标类别过滤：255=任意, 0=sandbag, 1=netball, 2=bear |
| `trk_min_confidence` | 70 | 最低置信度 (0-100) |
| `trk_confirm_frames` | 4 | 状态切换需连续确认的帧数 |
| `trk_stop_bottom_pct` | 95.0 | bbox 底边触发停车的画面百分比 |
| `trk_reverse_angle` | 30.0 | 目标丢失后回退角度 (deg) |

---

## 存根维护规则

以下模式确保 Pylance 在 `例程/` 上零错误：
- `from array import array` — 任何将 `'array'` 作为前向引用使用的 `.pyi` 中必须加上
- 所有 `help()` 方法加 `@staticmethod` — 示例调 `ClassName.help()`，从不调 `instance.help()`
- 引脚参数类型用 `Optional[str]` — 示例先将引脚变量初始化为 `None`，再赋值为字符串
- `info()` 始终是实例方法，不是静态方法

---

## 无构建/测试/检查命令

这是一个嵌入式 MicroPython 项目。没有本地构建、没有 test runner、没有 linter。代码在 VSCode 中编辑（Pylance 使用存根），然后通过 Thonny 上传到硬件执行。唯一的"验证"是在硬件上实际运行。

## 格式

Tab Size = 2 空格
