# 智能小车分步调试

> 入口技能：按流程逐状态调试比赛小车。每个状态一个测试脚本，只改测试脚本不改主代码（除非用户允许）。

## 调试流程

```
LEAVE(出库) → HUNT(搜索) → ALIGN(对位) → PUSH(推箱) → BACKOFF(后退) → HOME(回库)
```

每个状态验证通过后再进入下一个。

## ✅ LEAVE（出库）

**测试脚本**：`CODE/test_leave.py`
**项目代码**：`match_hunt.py:_tick_leave`

1. EXIT：直行锁航向，直到压黄线→离线（跨线）
2. SHIFT：根据发车位置横向平移（锁航向）
3. 看到目标 → 进 ALIGN/HUNT

## ✅ HUNT（搜索/追踪）

**测试脚本**：`CODE/test_hunt.py`
**项目代码**：`match_hunt.py:_tick_hunt`

1. SPIN：原地自旋搜索（360°反转），看到→停车确认→TRACK
2. TRACK：bearing PID 追踪接近
3. y2≥stage：yaw在接近方向±25°→PUSH(skip ALIGN)，否则→ALIGN

## ✅ ALIGN（绕行对位）

**测试脚本**：`CODE/test_align.py`
**项目代码**：`match_hunt.py:_tick_align`

1. TURN：yaw不对→绕前方轴(move(90°)=[小,小,大]) | yaw对→侧移居中+前推
2. 接触+yaw_ok+cx_ok → PUSH
3. 丢目标→扫弦→久丢→HUNT，超时→HUNT
4. yaw滞回防抖(1.5×)，最小绕行防MIN_DUTY卡死

## ✅ PUSH（推箱）

**测试脚本**：`CODE/test_push.py`
**项目代码**：`match_hunt.py:_tick_push`

1. DRIVE：全速前推+航向锁
2. CORRECT：PD侧移闭环纠偏(cx>50→右移)，减速前推12%
3. 黄线→BACKOFF，丢目标→HUNT，超时→HUNT

## ✅ BACKOFF（后退+自旋）

**测试脚本**：`CODE/test_backoff.py`
**项目代码**：`match_isr.py:step_backoff`

1. RETREAT：后退最少 800ms 离线
2. SPIN：闭环 PD 自旋 180°（±3°×5帧消抖），超时 3s
3. → HOME（得分达标）或 FWD→HUNT（继续推）

## ✅ HOME（回库）

**测试脚本**：`CODE/test_home.py`
**项目代码**：`match.py:_tick_home`

回库路径 (按 layout):
  layout=1(底边中): LEG1_TURN(180°) → LEG1_DRIVE(陀螺) → CROSS(1s) → DONE
  layout=2(左下角): LEG1_TURN(90°) → LEG1_DRIVE(陀螺) → 压线→BACKOFF→TURN(180°) → LEG2_DRIVE(陀螺+XB) → CROSS → DONE
  layout=3(右下角): 同上 y1=-90°

1. LEG1: 纯陀螺仪航向锁，LEG2: 陀螺 + XB 信标横向 PD 修正
2. LEG2 切换 `match_allow=[CLS_XB]`，只追踪信标
3. XB 确认：压黄线 + y2≥75%(信标够近) → CROSS → DONE
4. _spin_toward：PD 自旋到目标 ±10° + yaw_rate<30°/s + 消抖 8 帧
5. CROSS：过线后继续前进 1s 防停在线上
6. BACKOFF(Home内)：短退离线（≥800ms 后离线或超时 1.5s）

## MIN_DUTY 控制

- `MotionControl.setSpeed(duties, use_min_duty=True)` — 默认开启
- `MotorArbiter.write(cid, duties, use_min_duty=True)` — 同上
- **混合运动处关闭**（fwd+rot、fwd+side、side+rot）：传 `False`
- **纯自旋/纯直行**保持默认 `True`
- 原因：MIN_DUTY=7 把 1-6% 小修正拉到 7%，全向合成时失真严重

## 运动学（已验证）

- `move()` 公式正确（勿改！）：`[speed*(s+c), speed*(s-c), speed*(-2s)]`
- `move_forward(s)` = `move(s, 0)` = `[s, -s, 0]` **（手写版是错的）**
- `move_side(s)` = `move(s, -90)` = `[s/3, s/3, -2s/3]` = **右移** **（手写版是错的）**
- `move(s, 90)` = `[-s/3, -s/3, 2s/3]` = **左移** = `[小,小,大]` = 绕前方轴
- `move(d, 0)` = **向前**
- 电机极性：`[d,d,d]` 正→CW(yaw↓)，`yaw_actuation_sign=-1`

### 车体-摄像头坐标关系
- 车右移 → 画面物体左移 → **cx 减小**
- 车左移 → 画面物体右移 → **cx 增大**
- 所以：目标 cx<50（偏左）→ 车需**左移**才能居中 → lateral 应为负 → 公式 **`(cx-50)`**
- PUSH 跟随：`err_cx = cx-50`（物体偏右→车右移跟随，同向）
- **以上关系均为开环验证，不可直接用于闭环控制**

## 摄像头

### 模型
- 5 个模型：[1]iou/112 [2]iou/160 [3]siou/112 [4]siou/160 [5]ciou/160
- **选用 [2]**: 160x160, F1>96%, 三类零交叉混淆
- **后处理坑**：train.py --width/--height 覆盖训练参数，但 `add_post_node()` 读 config.cfg 写死的值（当时是 112x112），导致 160x160 模型加了 112 后处理 → OpenART `tf.load()` 报 "not a detect model"
- **修复**：改 config.cfg→160x160，删旧 pp 文件，重新 `add_post_node('[2]_yolo3_iou_smartcar_final.tflite')`，新 pp 文件 106864 bytes（和 [4]/[5] 一致，旧版 105992）

### CameraCode (OpenART)
- `camera._parse` 已解码：`d[0]=cls, d[1]=conf(0-31)`, `d[6]=cx%`, `d[9]=y2%`
- 热重启：检测循环收到数据→回0x02重握手
- **5 类全发送**：CLASS_NAMES=('sandbag','netball','bear','XB','brick')，CONF_MIN=0.50，MAX_OBJ=20
- **tft_test()**：独立模型测试，无 UART。`tf.detect` 返回**归一化坐标 0-1**，画框需 `int(x*img.width())`，`draw_rectangle((x,y,w,h), color, thickness)` 用**元组**传参
- 实际运行：6FPS，每帧 2-7 个检测，画框正常

### MCU 端 (main.py)
- `select_target(detections, cfg, allow, target_class)` — allow 列表过滤类别
- HUNT/ALIGN/PUSH: `match_allow=[0,1,2]`（只追踪比赛三目标）
- HOME LEG2: `match_allow=[3]`（只追踪 XB）
- `brick_blocking` flag: `_build_sensors` 中扫描原始帧，brick 同 cx 方向 + y2 更大 → 挡路
- PUSH EVADE: brick 挡路→降速 25% + 侧移 50% 绕开

### 模型对比脚本
- `compare_model.py`：随机抽样→模型推理→逐框 IoU 匹配→per-class P/R/F1 + 混淆矩阵 + 可视化
- PC 端验证用，确认模型在数据集上的性能

## 配置

- `config.py`：纯模块级变量，`import config as cfg`，无 JSON，无类
- 构建脚本 `build_flash.py --strip-log` 剥日志
- 类别：0=沙袋 1=网球 2=熊 3=XB(信标) 4=brick(干扰)
- 发车位置：1=底边中 2=左下角 3=右下角
- 推箱方向：网球→0°, 沙包→90°, 熊→-90°
- 推物顺序：[0,1,2]（沙包→网球→熊）不可推 brick
- HOME LEG2: 陀螺航向锁 + XB 横向 PD（kp=0.6 kd=0.5, y2≥75 确认）

## 内存

- IMU ticker 必须 `capture_list(imu.raw)`
- MIN_DUTY=7 提升小占空比；混合运动 `use_min_duty=False` 关掉
- 内联常量到注释，构建脚本去除
- 整车运行时内存低至 3.6KB，GC 可恢复但需关注

## 规则

- 每次回答简短，只告诉下一步
- 只改测试脚本，不改主代码（除非允许）
- 调好的参数记录并合入项目
