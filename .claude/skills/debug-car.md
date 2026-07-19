# 智能小车分步调试

> 入口技能：按流程逐状态调试比赛小车。每个状态一个测试脚本，只改测试脚本不改主代码（除非用户允许）。

## 调试流程

```
LEAVE(出库) → HUNT(搜索) → ALIGN(对位) → PUSH(推箱) → BACKOFF(后退) → HOME(回库)
```

每个状态验证通过后再进入下一个。

## ✅ LEAVE（出库）— 已完成

**测试脚本**：`CODE/test_leave.py`

**LEAVE 逻辑**（对应 `match_hunt.py:_tick_leave`）：
1. EXIT：直行锁航向，直到压黄线→离线（跨线）
2. SHIFT：根据发车位置横向平移（锁航向），超时→HUNT
3. 看到目标 → 锁定类别，进 ALIGN 或 HUNT

**已确认的关键参数**：
- 电机极性：`[d,d,d]` 正占空比 → CW（yaw↓），`yaw_actuation_sign = -1`
- 运动学：`move(speed, 0)` = `[s, -s, 0]` 前进，`move(speed, -90)` = `[s, s, -2s]` 右移
- 轮1 反号：原始运动学公式对轮1需取反（硬件接线方向）
- 发车位置：1=底边中 2=左下角(右移) 3=右下角(左移)
- 推箱方向：网球→0°, 沙包→90°, 熊→-90°
- config：纯模块级变量 `import config as cfg`，无 JSON 无类
- 摄像头：`camera._parse` 已解码 `d[0]=cls, d[1]=conf(0-31)`
- IMU ticker 必须 `capture_list(imu.raw)` 否则航向锁不工作

## ✅ HUNT（搜索/追踪）— 已完成

**测试脚本**：`CODE/test_hunt.py`

**HUNT 逻辑**（对应 `match_hunt.py:_tick_hunt`）：
1. SPIN：原地自旋搜索（每360°反转），看到目标→停车确认
2. TRACK：确认 N 帧后→视觉 bearing PID 追踪接近
3. y2≥stage 后判断 yaw：
   - yaw 已在推箱接近方向（±25°）→ 直进 PUSH（跳过 ALIGN）
   - yaw 偏差大 → ALIGN 绕行

**推箱方向**：
- cls=0(沙包)→push 90°, approach -90°
- cls=1(网球)→push 0°, approach 180°
- cls=2(熊)→push -90°, approach 90°

**验证结果**：SPIN→TRACK→yaw判断→ALIGN/PUSH 流程正常。
摄像头模型目前只对沙包类(cls=0)敏感，需后续优化模型。

## ✅ ALIGN（绕行对位）— 已完成

**测试脚本**：`CODE/test_align.py`

**ALIGN 逻辑**（对应 `match_hunt.py:_tick_align`）：
1. TURN: yaw不对→绕前方轴(move(90°)=[小,小,大]) | yaw对→侧移居中+前进
2. yaw_ok+cx_ok+接触 → PUSH (无CLOSE)
3. 丢目标→扫弦找→久丢→HUNT

**已验证**：go(-90)/go(0) TURN→PUSH成功, go(90)进死角→HUNT

## 🔜 下一状态：PUSH（推箱）

## 相关技能

- [[motor-debug]] — 电机运动学、极性、架构
- [[pd-tune]] — PD 参数整定
- [[imu-tune]] — IMU 标定与参数
- [[mag-calibrate]] — 磁力计校准

## 规则

- **每次回答简短**，只告诉用户下一步做什么
- **只改测试脚本**，不改主代码（除非用户明确允许）
- 观察用户反馈的现象，诊断问题后再给下一步指令
- 当前状态通过后再进入下一状态
- 现场调好的参数及时记录到本技能，合入项目代码
