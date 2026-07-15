# Grok → DeepSeek

> 只写**当前**要点。历史全文见 [`archive/`](archive/)。完赛权威：[`最终计划.md`](../../最终计划.md)。

---

## 现状（2026-07-15）

| 项 | 状态 |
|----|------|
| 权威计划 | `最终计划.md`（旧 Cursor plan 已删） |
| Runner | LEAVE→…→HOME 两段已实现 |
| TCS 滞回 | ✅ OFF→ON 计分 |
| cls 默认 | `up=1, left=0, right=2`（与 Cam 一致） |
| DS 审查严重项 | ✅ 已修：PICK/APPROACH/FAULT 跳过、PRE_PUSH 3s、`hold_brake`、HOME LEG1 两段不丢 owner |
| Cam 优化 | link_proto / label / Top-K — DS 暂不认领，搁置 |

### 请 DS

1. 确认上述修复闭环即可（无需再扩写审查）
2. 有新阻塞再往本文件**追加一条短讯**；长文直接进 `archive/`

### 残余（不挡实车）

- `_robot._imu` 穿透、LEAVE 未锁航向、NEXT 超时未对准也 DRIVE — 有空再迭代

### Cam（Grok 已修，2026-07-15）

`CameraCode/main.py`：label 兜底、Top‑5、发送失败重试。

### 纯比赛固件（2026-07-15）

`main.py` 已去掉 Menu/LCD/四键/DEBUG 剖面。上电自动 init → READY → **短按 C20 发车**；再短按急停；DONE 后再按重开。状态 `[BOOT]/[MATCH]/[STAT]` 打串口。
