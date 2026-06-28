# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

MicroPython firmware development for **NXP RT1021 (i.MX RT1021)** using Seekfree (逐飞科技) core boards. Code runs on bare metal — no OS, no CPython, no standard library beyond MicroPython builtins. `.py` files are uploaded to the board's flash via USB and executed by the on-board MicroPython interpreter.

**Tooling**: Thonny IDE for flashing/REPL; VSCode for editing with the `.stubs2.2.0/` type stubs. Stubs target **firmware v2.2.0**.

## Repository layout

```
.stubs2.2.0/         ← VSCode/Pylance type stubs (this repo's primary artifact)
  machine/           ← Pin, ADC, PWM, UART, SPI, I2C, SoftI2C, BOARD_TYPE, BOARD_VERSION
  seekfree/          ← IMU660RX, IMU963RX, MOTOR_CONTROLLER, KEY_HANDLER, IPS200PRO, DL1X, TSL1401, WIRELESS_UART, WIFI_SPI
  smartcar/          ← ticker, encoder, ADC_Group
  display/           ← LCD_Drv, LCD
  time/ os/
.vscode/             ← Pylance stubPath + extraPaths config
CODE/                ← User application (the robot firmware)
  main.py            ← Entry point: hardware init → ticker → main loop
  Motor.py           ← MotionControl: 3× DRV8870DDAR omni-wheel drive
  Menu.py            ← Ring menu system with IPS200 display
  imu.py             ← IMU660RX + Madgwick AHRS attitude estimation
  config.py          ← JSON config persistence on /flash
例程/                ← Official Seekfree demos — DO NOT EDIT, used as API reference
CameraCode/          ← OpenART Plus camera coprocessor: UART protocol, test tools, OpenART demos
  camera_test.py     ← OpenART-side communication test (4 test modes)
  (MCU)test_camera.py ← RT1021-side paired test (loopback + start)
  摄像头测试流程.md    ← Step-by-step test guide
  摄像头文件.bat       ← SD card file management for OpenART
  【例程】OpenART Plus例程/ ← OpenART Plus official examples — READ-ONLY reference
Model/               ← PC-side YOLOv3-tiny training pipeline (Python 3.10, TF2.x)
  yolo3_smartcar/    ← training scripts, model, TFLite output
RT1021-MicroPython 固件接口说明.pdf   ← Official firmware API documentation
SeekFreeHelp.md      ← Module pinout/API quick reference
```

## Critical constraints

- **`例程/` is read-only.** These are official Seekfree examples. When stubs conflict with example patterns, fix the **stubs**, not the examples.
- **Only import what exists.** No `argparse`, `logging`, `unittest`, `typing` (runtime), `pathlib`, `threading`, etc. MicroPython builtins only.
- **File I/O via `io.open()` and `os`.** Flash filesystem at `/flash`. Use absolute paths. Do NOT open/close files in loops — flash has limited write cycles.
- **Pin names are strings:** `'C4'`, `'B27'`, `'D9'`, not integers.
- **`gc.collect()`** must be called regularly in main loops.
- **UART buffer is 63-64 bytes.** Both OpenART and RT1021 MicroPython firmware have an internal UART ring buffer of ~64 bytes. `uart.write()` silently truncates writes >63 bytes; `uart.any()` never reports >63. Workaround: chunk writes ≤60 bytes with 2ms delays, and read incrementally (never `while uart.any() < N` for N>63). See `CameraCode/camera_test.py` → `write_chunked()` and `recv_frame()`. 460800 baud ≈ 46 KB/s → 60 bytes ≈ 1.3ms on wire.

## MicroPython-specific patterns

### Global runtime constants
`BOARD_TYPE` and `BOARD_VERSION` are available globally without import — they are `str` values like `'RT1021_144P_BTB'`, `'RT1021_144P_2P54'`, `'RT1021_100P_2P54'`.

### Sensor data buffers are auto-refreshing
`imu.get()`, `key.get()`, `encoder.get()` each return a **linked buffer** — call `.get()` once, keep the returned list reference, and its contents refresh automatically on each ticker-driven `.capture()`. Do NOT call `.get()` every tick.

### ticker (smartcar module)
- `ticker(id)` — PIT channel [0, 3]. Tick period set by `start(ms)` (min 5ms).
- `capture_list(*modules)` — registers sensors (ADC_Group, encoder, KEY_HANDLER, IMU*, DL1X, TSL1401) for automatic `.capture()` on each tick.
- `callback(handler)` — handler receives the ticker instance as its single argument.

### KEY_HANDLER
- `get()` returns `[k1, k2, k3, k4]` — 0=none, 1=short press, 2=long press.
- `clear(index)` — index is **1-based** (1–4). `clear()` clears all.

### IMU sensors
- **IMU660RX** (6-axis): `get()` → `[ax, ay, az, gx, gy, gz]`. Hardware quaternion available via `quar_rate` param (requires INT2 on D19, incompatible with `ticker.capture_list`).
- **IMU963RX** (9-axis): `get()` → `[ax, ay, az, gx, gy, gz, mx, my, mz]`. `quar_rate` only supports `RATE_DISABLE` — all attitude estimation must be done in software.
- Both raise `ValueError: Module init fault.` if hardware not connected — always wrap init in try/except.
- Raw-to-physical: acc ±8g → 4096 LSB/g; gyro ±2000dps → 16.384 LSB/(deg/s); mag ≈ 16 LSB/uT (BMM150).

### MOTOR_CONTROLLER
- Duty range `[-10000, 10000]`. Pin-pair constants: `PWM_C30_DIR_C31` (PWM+DIR), `PWM_C30_PWM_C31` (PWM+PWM), etc.
- `duty()` with no arg returns current; with arg sets and returns.

### IPS200 display
- `LCD_Drv.LCD200_TYPE` + `LCD(spi_index, baudrate, dc_pin, rst_pin, lcd_type)`.
- Raw drawing via `lcd.str24(x, y, text, color)`, `lcd.clear(color)`, `lcd.mode(n)`.
- `IPS200PRO` is the GUI widget library (pages, labels, tables); `LCD` is the raw drawing layer. Only font sizes 16/20/24 support Chinese.

## OpenART Plus — camera coprocessor communication

OpenART Plus is a separate NXP RT1064 MCU with a camera sensor + NPU, running its own MicroPython firmware. It connects to RT1021 via **UART (LPUART6: TX=D20 → OpenART RX, RX=D21 ← OpenART TX, 460800 baud)** and sends object detection results as framed binary packets.

### Protocol frame

```
[0xAA] [CMD:1B] [LEN:1B] [DATA:N bytes] [CRC:1B]
```

CMD 0x0F=heartbeat, 0x10=detection results, 0x20=UTF-8 text, 0xF0=loopback

### Detection payload format

```
[num:1B] [cls:1B score:1B x1:1B y1:1B x2:1B y2:1B] × N
```
All coordinates normalized to 0-255 (actual = raw/255 × image_dims). Score 0-100 (%).

### Escape mechanism

0xAA→0xBB 0x00, 0xBB→0xBB 0x01. Applied to payload before framing to prevent false sync markers.

### OpenART files

Model + labels go on SD card:
- `yolo3_iou_smartcar_final_with_post_processing.tflite` — the detection model
- `cmm_cfg.csv`, `cmm_load.py` — SD card boot files
- `labels_animal_fruits.txt` — class names (if using classification model)

### OpenART MicroPython differences

- UART import may be `from machine import UART` or `from pyb import UART` — `camera_test.py` tries both
- `import tf` loads the TensorFlow Lite runtime (OpenART-only, not available on RT1021)
- `sensor` module to control the camera (reset, snapshot, set_pixformat, etc.)
- `image` module for drawing (draw_rectangle, draw_string)

### Test workflow

See `CameraCode/摄像头测试流程.md` for full guide. Quick summary:
1. OpenART: `import camera_test; camera_test.run(test_number=1)` — UART loopback (short TX-RX)
2. OpenART: `camera_test.run(test_number=4)` + RT1021: `import test_camera; test_camera.start()` — MCU comm test
3. OpenART: `camera_test.run(test_number=3)` + RT1021: `test_camera.start()` — full detection + comm

## Model training pipeline (PC-side)

YOLOv3-tiny trained with TensorFlow 2.x on Python 3.10. Not MicroPython — this runs on the PC and produces `.tflite` files deployed to OpenART's SD card.

### Data
- `Model/data/` — VOC format: `JPEGImages/*.jpg` + `Annotations/*.xml`
- `Model/data2500/` — expanded dataset (2500 images)
- Classes: netball (0), sandbag (1), bear (2)

### Training workflow (all via `Model/Model.bat`)
1. `voc_convertor.py` — VOC XML → train_data.txt
2. `kmeans.py` — generate anchor boxes → yolo3_anchors.txt
3. `train.py` — train ResNet-based YOLOv3-tiny, outputs `.tflite`
4. `evaluate.py` — mAP evaluation
5. `detect.py` — single-image inference test (`-model X.tflite -image test.jpg`)

### Auto-labeling
`Model/auto_label.bat` runs `auto_label.py` — use a trained model to pre-label new images (semi-automated dataset expansion). Outputs VOC XML to `Annotations/`.

### Annotation viewer
`Model/view.bat` runs `view_annotations.py` — reads VOC XML + JPEG images and generates an HTML gallery for browsing annotations. Opens in browser automatically.

### Key files
| File | Purpose |
|------|---------|
| `config.cfg` | voc_folder path, class names, training hyperparams |
| `train_data.txt` | Generated annotation list (image_path x1,y1,x2,y2,class_id ...) |
| `yolo3_anchors.txt` | K-means computed anchor boxes |
| `yolo3_iou_smartcar_final.tflite` | Trained model (no post-processing) |
| `yolo3_iou_smartcar_final_with_post_processing.tflite` | Model with NMS baked in → deploy this to OpenART SD card |
| `tflite_add_post_processing.py` | Adds NMS/detection post-processing to a plain TFLite model |

## CODE/main.py architecture

Multi-file application, loaded onto `/flash` and executed by the on-board MicroPython interpreter:

| Class | Role |
|-------|------|
| `MotionControl` | DRV8870DDAR dual-PWM half-bridge × 3 omni-wheels. Speed range [-100, 100] %. `stop()`=coast(LOW/LOW), `brake()`=brake(HIGH/HIGH). `sleep_us(76)` between PWM writes as DRV8870 dead-time. `move(speed, angle)` = omni inverse kinematics. |
| `ImuSensor` | IMU660RX wrapper with Madgwick AHRS fusion: startup gyro bias calibration → online EMA bias tracking → quaternion attitude estimation. Exposes `get_yaw()/pitch()/roll()`, `recalibrate()`, `is_calibrated`, `bias_dps`. |
| `DisplayDriver` | Ring-menu renderer on IPS200. Left arc layout (focused item centered, cosine size falloff), right info panel (title + value). |
| `Menu` | Ring menu controller: `goto()`, `jump_to()`, `handle_input()`, `update_display()`. Supports AdjustItem for editable parameters with auto-save to `/flash/config.json`. |
| `config` | JSON config persistence: atomic write (tmp + rename), auto-creates defaults on first boot. |
| `HeadingController` | PID heading closed-loop via IMU yaw feedback. Two modes: `straight` (lock initial heading + forward speed via omni kinematics), `lock` (rotate in place to target angle). Uses `HeadingPID` — incremental PID with deadband + back-calculation anti-windup. Dependencies: `MotionControl.move()`, `ImuSensor.get_yaw()`, config dict for gain hot-reload. `update()` called each main loop iteration. |
| Main (module-level) | Init hardware → build menu → start ticker (10ms) → main loop reads 4 raw GPIO pins for key input. |

### Config keys (persisted to `/flash/config.json`)

| Key | Default | Used by |
|-----|---------|---------|
| `target_speed` | 50.0 | Forward speed in straight mode |
| `heading_kp`, `heading_ki`, `heading_kd` | 2.0, 0.0, 0.0 | Heading PID gains |
| `heading_max_correction` | 50.0 | PID output clamp — max rotation duty added to wheels |
| `heading_deadband` | 1.0 | Deadband in degrees — `|error| < deadband` → zero output |

## Stub maintenance rules

These patterns keep Pylance at zero errors across `例程/`:
- `from array import array` — required in any `.pyi` that uses `'array'` as forward-reference.
- `@staticmethod` on all `help()` methods — examples call `ClassName.help()`, never `instance.help()`.
- `Optional[str]` for pin parameters — examples initialize pin vars to `None` then assign strings.
- `info()` is always an instance method, not static.

## No build/lint/test commands

This is an embedded MicroPython project. There is no local build, no test runner, no linter. Code is edited in VSCode (with Pylance using the stubs), then uploaded to the board via Thonny for execution. The only "verification" is running on hardware.

## Format

Tab Size = two space