# 智能小车分步调试

> 入口技能：按流程逐状态调试比赛小车。每个状态一个测试脚本，只改测试脚本不改主代码（除非用户允许）。

## 调试流程

```
LEAVE(出库) → HUNT(搜索) → ALIGN(对位) → PUSH(推箱) → BACKOFF(后退) → HOME(回库)
```

每个状态验证通过后再进入下一个。

## 当前状态：LEAVE（出库）

**测试脚本**：`CODE/test_leave.py`

**LEAVE 逻辑**（对应 `match_hunt.py:_tick_leave`）：
1. 看到目标 → 锁定类别，进 ALIGN 或 HUNT
2. 超时 → 进 HUNT
3. 否则 → 直行 + 航向锁定 (`_write_move_locked`)

**验证步骤**：

### Step 1: 电机极性验证
```python
>>> import test_leave
# 等 IMU 标定完成
>>> spin_test(30)    # [30,30,30] 三轮同速自旋
# yaw↑=CCW, yaw↓=CW → 记录极性方向
>>> fwd_test(30)     # move_forward，看车实际朝哪走
```

### Step 2: 直行+航向锁定
```python
>>> go(0, 50)        # hold_yaw=0°, duty=50%
>>> run(20)          # 20Hz 连续跑
# 观察：是否走直线、yaw 误差是否收敛到 ±5° 以内
>>> mon()            # 查看当前状态
>>> Ctrl+C 停止
```

## 相关技能

- [[motor-debug]] — 电机最低占空比、极性、架构
- [[pd-tune]] — PD 参数整定
- [[imu-tune]] — IMU 标定与参数
- [[mag-calibrate]] — 磁力计校准

## 规则

- **每次回答简短**，只告诉用户下一步做什么
- **只改测试脚本**，不改主代码（除非用户明确允许）
- 观察用户反馈的现象，诊断问题后再给下一步指令
- 当前状态通过后再进入下一状态
