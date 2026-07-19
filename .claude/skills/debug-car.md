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

## 🔜 BACKOFF（后退）— 测试中

**测试脚本**：`CODE/test_backoff.py`
**项目代码**：`match_isr.py:step_backoff`

1. RETREAT：后退 N ms 离线
2. SPIN：开环自旋 170°→HOME或HUNT(FWD)

## 🔜 HOME（回库）

**项目代码**：`match.py:_tick_home`

## 运动学（已验证）

- `move()` 公式正确（勿改！）：`[speed*(s+c), speed*(s-c), speed*(-2s)]`
- `move_forward(s)` = `move(s, 0)` = `[s, -s, 0]` **（手写版是错的）**
- `move_side(s)` = `move(s, -90)` = `[s/3, s/3, -2s/3]` = 右移 **（手写版是错的）**
- `move(s, 90)` = `[-s/3, -s/3, 2s/3]` = 左移 = `[小,小,大]` = 绕前方轴
- 电机极性：`[d,d,d]` 正→CW(yaw↓)，`yaw_actuation_sign=-1`
- 侧移修正：`(50-cx)` 配合 `move_side` = 正确方向
- CORRECT推箱侧移：`(cx-50)` = 跟随物体方向

## 摄像头

- camera._parse 已解码：`d[0]=cls, d[1]=conf(0-31)`
- CameraCode 热重启：检测循环收到数据→回0x02重握手
- 模型目前只区分沙包类(cls=0)，需优化

## 配置

- `config.py`：纯模块级变量，`import config as cfg`
- 无 JSON，无类，构建脚本 `--strip-log` 剥日志
- 发车位置：1=底边中 2=左下角 3=右下角
- 推箱方向：网球→0°, 沙包→90°, 熊→-90°

## 内存

- INU ticker 必须 `capture_list(imu.raw)`
- MIN_DUTY=7 提升小占空比，可能改运动方向
- 内联常量到注释，构建脚本去除

## 规则

- 每次回答简短，只告诉下一步
- 只改测试脚本，不改主代码（除非允许）
- 调好的参数记录并合入项目
- 当前状态通过后进下一状态
