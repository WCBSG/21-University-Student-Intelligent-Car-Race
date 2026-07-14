# 智能车固件重构 — 设计文档

> 此文件由 Grok 和 DeepSeek 共同维护。Git 跟踪。只写决策，不写过程。

## 架构

```
main(~50Hz): drain Intent → sense → robot.tick → render
Robot: 唯一 Mode + MotorArbiter（同时仅一人写电机）
Menu: 只发 Intent，不控电机 / 不阻塞
proto: CODE/link/proto.py 单份，同步到 CameraCode/link_proto.py
```

## 目录（实际，相对 CODE/）

```
app/     intent.py, mode.py, fsm.py
ctrl/    arbiter.py, heading_mode.py, track.py
link/    proto.py, camera_rx.py          ← DS 步3
config.py, Motor.py, imu.py, Menu.py, main.py
CameraCode/link_proto.py + main.py       ← 协议同步副本
```

（目标树中的 `hal/` `ui/` 迁移可在步5后整理，非阻塞。）

## 状态转移表（最终版）

```
           IDLE  HDG  SEARCH TRACK COMPLETE FAULT
GO_STRAIGHT HDG  HDG  -      -     HDG      -
LOCK_YAW    HDG  HDG  -      -     -        -
START_TRACK SRCH SRCH -      -     -        -
ABORT       IDLE IDLE IDLE   IDLE  IDLE     IDLE
STOP        IDLE IDLE IDLE   IDLE  IDLE     IDLE
target_ok   -    -    TRACK  -     -        -
target_lost -    -    -      SRCH‡ -        -
bbox_95%    -    -    -      CMPL  -        -
cam_timeout FAULT FAULT FAULT FAULT FAULT   -
RECONNECT   IDLE -    -      -     -        IDLE
═════════════════════════════════════════════════════
‡ TRACK→SEARCH 触发 SEARCH.phase='reverse'（非独立状态）
守卫: START_TRACK 需 imu.is_calibrated
      SEARCH/TRACK 中 GO_STRAIGHT/LOCK 禁止（须先 STOP）
去抖: target_ok/lost 用 Debouncer，按相机帧计数
      默认 cfg.tracking.confirm_frames / lost_frames = 4
确认期间: SEARCH 有目标时停转（写 [0,0,0]），禁止全速自旋
```

## 接口契约（锁定）

```text
Config:     cfg.heading / cfg.tracking_bearing / cfg.tracking.*
            引用模式，Controller 不缓存副本，改字段即生效
Arbiter:    acquire(id) / release(id) / write(id, duties) / force_brake()
Intent:     ABORT|STOP|START_TRACK|GO_STRAIGHT|LOCK_YAW|RECONNECT
            每拍 drain 全部；含 ABORT → 只执行 ABORT，清空其余
FSM:        robot.handle(intent, arg=None) → bool
            robot.tick(dt, sensors) → None
            robot.on_camera_frame(has_target, y2=0.0)  # 或 tick 内 new_frame 触发
            sensors = {
              new_frame, has_target, target, y2, cam_timeout
            }
Mode:       enter() / update(dt, sensors) → None / exit()
            不自行转移；owner id = 状态名（HDG/SEARCH/TRACK/…）
CameraRx:   poll() → DetectionFrame | None
            timed_out → bool
            空帧 DetectionFrame(num=0) 清目标
RECONNECT:  robot.reconnect_pending；仅 IDLE|FAULT 可发起；成功→IDLE
build_robot(arbiter, cfg, imu) → RobotFSM（注册全部 Mode）
```

## 不做

- EventBus / 回调嵌套
- ACK / 重传
- 菜单内同步长阻塞 handshake（步5改为分拍）
- update_pid_gains() 扩散（Config 引用消灭它）
- 改 Motor.move 运动学 / Madgwick / DRV8870 驱动

## 落地步骤

| 步 | 内容 | 状态 |
|----|------|------|
| 1 | Config 引用化 + PidGains | ✅ |
| 2 | MotorArbiter + Intent + FSM 骨架 | ✅ |
| 3 | 合并双端 proto + CameraRx | ✅ DS |
| 4 | Tracker 迁入 Mode + FSM tick 补全 | ✅ Grok |
| 5 | Menu 削干净 + CameraRx 接入 main + IMU 双缓冲 + 握手分拍 | ✅ DS |

## 成功标准

- 结构上无法双写电机
- ABORT 必全停
- 目标消失 ≤1 发送周期可感知
- 新行为 = 加 Mode 类 + 表里加一行
