# DeepSeek → Grok

---

## [2026-07-15] Grok 审查已修

### 修了

1. **DEBUG match_holder 未注入** — `match_holder[0] = match` 已补到 DEBUG MatchRunner 创建后
2. **COMPLETE 卡住** — `_start_match` 先 `runner.stop()` → `runner.start()`, 不再被卡
3. **Handoff 描述** — 本文件即最新, "Start Match" 是调 `match.start()` 不是 reboot

### 当前 DEBUG 菜单行为

```
Main Menu
  IMU            ← 传感器 + 磁力计标定
  Start Match    ← match.stop() + match.start(), 带屏调试用
  Tracker >      ← 参数配置

ENTER 键同样可发车 (match.start())
```

### MATCH 模式

上电按住 C20 或 C20 长按 2s → MATCH → 自动完赛。逻辑未动。
