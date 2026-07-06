"""
config.py — 集中持久化模块

全局 config 字典是唯一持久化数据源，文件读写仅由此模块负责。
save_config() 采用原子写入（tmp + rename），防止写一半断电损坏配置。
"""

import json, os

CONFIG_FILE = "/flash/config.json"

DEFAULT_CONFIG = {
  # 速度参数
  "target_speed": 50.0,

  # 角度阈值
  "angle_threshold": 10.0,

  # 航向闭环 PID
  "heading_kp": 2.0,
  "heading_ki": 0.0,
  "heading_kd": 0.0,
  "heading_max_correction": 50.0,
  "heading_deadband": 1.0,

  # 视觉跟踪 — Bearing PI
  "trk_bearing_kp": 1.5,
  "trk_bearing_ki": 0.05,
  "trk_bearing_kd": 0.0,
  "trk_bearing_max": 60.0,
  "trk_bearing_db": 0.02,

  # 视觉跟踪 — Speed
  "trk_approach_speed": 15.0,
  "trk_search_speed": 15.0,

  # 视觉跟踪 — Target selection
  "trk_target_class": 7,
  "trk_min_confidence": 22,

  # 视觉跟踪 — Thresholds
  "trk_confirm_frames": 20,   # 50Hz × 20 = 400ms ≈ 4个相机帧
  "trk_stop_bottom_pct": 95.0,
  "trk_reverse_angle": 30.0,
}

# 全局配置字典 — 模块级单例
config = {}


def load_config():
  """启动时调用。从 /flash/config.json 读取，失败则用默认值并自动创建文件。"""
  global config
  try:
    with open(CONFIG_FILE,"r") as f:loaded = json.load(f)
    # 用加载值更新 config，缺失的键保留默认值
    config.update(DEFAULT_CONFIG)
    config.update(loaded)
  except:
    config.update(DEFAULT_CONFIG)
    save_config()


def save_config():
  """原子写入：先写 .tmp，再 rename。仅在必要时调用。"""
  tmp=CONFIG_FILE+".tmp"
  with open(tmp,"w") as f:json.dump(config, f)
  os.rename(tmp,CONFIG_FILE)
