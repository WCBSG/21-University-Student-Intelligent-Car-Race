import math
from time import ticks_ms, ticks_diff
from config import config, save_config

# 常用颜色
WHITE = 0xFFFF;YELLOW = 0xFFE0;GREEN = 0x07E0;CYAN = 0x07FF;BLACK = 0x0000;GRAY = 0x7BEF

# =============================================================================
#                              菜单项定义
# =============================================================================

class MenuItem:
  def __init__(self, text, action=None, get_value=None):
    self.text = text
    self.action = action          # callable(menu, item) 或 None
    self.get_value = get_value    # callable() -> str 或 None


class AdjustItem:
  def __init__(self, text, get_value, set_value,
               min_val, max_val, step, persistent=True, formatter=None):
    self.text = text
    self.get_value = get_value      # callable() -> float
    self.set_value = set_value      # callable(new_val)
    self.min_val = min_val
    self.max_val = max_val
    self.step = step
    self.persistent = persistent
    self.formatter = formatter if formatter else (lambda v: str(v))


class MenuPage:
  def __init__(self, id, name, items, on_enter=None, on_exit=None, refresh_ms=None):
    self.id = id
    self.name = name
    self.items = items
    self.on_enter = on_enter    # callable(menu) 或 None
    self.on_exit = on_exit      # callable(menu) 或 None
    self.refresh_ms = refresh_ms  # 动态页刷新间隔 (None=仅输入时刷新)


# =============================================================================
#                              页面注册表
# =============================================================================

_PAGES_BY_ID = {}   # page_id -> MenuPage  (O(1) 查找)


def get_page(page_id):
  """按 ID 获取页面。未找到返回 None。"""
  return _PAGES_BY_ID.get(page_id)


def _register(page):
  """注册页面到全局注册表。"""
  _PAGES_BY_ID[page.id] = page


# =============================================================================
#                             显示驱动
# =============================================================================

class DisplayDriver:
  """环形菜单渲染器。持有 LCD 实例，提供 render(...) 方法。"""

  # 文本宽度缓存（类级别，跨实例共享）
  _width_cache = {}
  _width_cache_max = 32

  def __init__(self, lcd, W=320, H=240):
    self.lcd = lcd
    self.W = W
    self.H = H

  @staticmethod
  def _map_font(size):
    """相对字号 → (LCD 绘制方法名, 字体高度)"""
    if size >= 14:
      return ("str24", 24)
    elif size >= 11:
      return ("str16", 16)
    else:
      return ("str12", 12)

  @classmethod
  def _text_width(cls, text, font_h):
    """估算文本像素宽度（带缓存）。ASCII 约 font_h//2，CJK 约 font_h。"""
    key = (text, font_h)
    w = cls._width_cache.get(key)
    if w is not None:
      return w
    w = 0
    for ch in text:
      if ord(ch) < 128:
        w += font_h // 2
      else:
        w += font_h
    if len(cls._width_cache) < cls._width_cache_max:
      cls._width_cache[key] = w
    return w

  def _draw_text(self, text, cx, cy, size, color):
    """以 (cx, cy) 为中心绘制文本。"""
    if not text:
      return

    method_name, font_h = self._map_font(size)
    tw = self._text_width(text, font_h)

    x = cx - tw // 2
    y = cy - font_h // 2
    if x < 0:
      x = 0
    if y < 0:
      y = 0

    draw = getattr(self.lcd, method_name)
    draw(x, y, text, color)

  def render(self, title, items, value, edit_mode):
    """
    消费 Menu.update_display() 生成的渲染数据，绘制完整界面。

    参数（全部为位置参数，消除每帧 dict 分配）：
      title     — 页面标题 str
      items     — 弧位列表 [(text, x, y, size, focused), ...]
      value     — 右侧面板数值 str
      edit_mode — 是否编辑模式 bool
    """
    self.lcd.clear(BLACK)

    # 左侧（38.2%）：环形菜单项
    for text, x, y, size, focused in items:
      color = YELLOW if focused else WHITE
      self._draw_text(text, x, y, size, color)

    # 黄金分割分隔线
    sep_x = int(self.W * 0.382)
    self.lcd.line(sep_x, 10, sep_x, self.H - 10, color=GRAY, thick=1)

    # 右侧面板（61.8%）中心 x
    rx = sep_x + (self.W - sep_x) // 2

    # 右侧上方：页面标题
    if title:
      self._draw_text(title, rx, 24, 16, CYAN)

    # 右侧下方：数值区
    if value:
      self._draw_text(value, rx, 120, 18, WHITE)

    # 编辑模式指示
    if edit_mode:
      self._draw_text(">", sep_x + 15, 120, 16, GREEN)
      hint = "+/-  ENTER:OK  BACK:Cancel"
      self._draw_text(hint, rx, 200, 9, GRAY)


# =============================================================================
#                              Menu 控制器
# =============================================================================

class Menu:
  """
  环形菜单控制器。

  公开方法（4 个）：
    goto(page, focus_index)   — 切换到指定页面
    jump_to(page_id, focus)   — 按 ID 跳转
    handle_input(key)         — 处理按键 (UP/DOWN/ENTER/BACK)
    update_display()          — 计算布局 + 刷新屏幕
  """

  def __init__(self, display_callback,
               W=320, H=240, R=None, Cx=None, Cy=None,
               step_angle=18, max_visible=5, base_size=16):

    self.display = display_callback

    self.W = W
    self.H = H
    self.R = R if R is not None else int(W * 0.7)
    # Cx 默认：弧位视觉中心对齐左侧 38.2% 区域的中央
    self.Cx = Cx if Cx is not None else int(W * 0.191 - self.R * 0.904)
    self.Cy = Cy if Cy is not None else H // 2
    self.step_angle = step_angle
    self.max_visible = max_visible
    self.base_size = base_size

    self.current_page = None
    self.focus_index = 0
    self.edit_mode = False
    self.edit_item = None
    self.edit_temp_val = 0.0

    # —— 预计算弧位坐标（消除每帧 cos/sin）——
    half = (self.max_visible - 1) // 2
    self._arc_slots = []
    for j in range(self.max_visible):
      offset = j - half
      theta = offset * self.step_angle
      rad = math.radians(theta)
      x = int(self.Cx + self.R * math.cos(rad))
      y = int(self.Cy + self.R * math.sin(rad))
      size = max(8, self.base_size - int(abs(theta) * 0.15))
      self._arc_slots.append((x, y, size))

    # —— 脏标记（跳过空闲时全屏重绘）——
    self._dirty = True
    self._last_render_ticks = 0

  # ——————————————————————————————————————————————————————————
  #                      页面导航
  # ——————————————————————————————————————————————————————————

  def goto(self, page, focus_index=0):
    """切换到指定页面。先切 current_page 再调旧 on_exit，避免递归。"""
    old_page = self.current_page
    self.current_page = page
    self.focus_index = focus_index
    self.edit_mode = False
    self.edit_item = None
    self._dirty = True

    if old_page and old_page.on_exit:
      old_page.on_exit(self)

    if page and page.on_enter:
      page.on_enter(self)

  def jump_to(self, page_id, focus_index=0):
    """按页面 ID 跳转。无效 ID 静默忽略。"""
    target = get_page(page_id)
    if target:
      self.goto(target, focus_index)

  # ——————————————————————————————————————————————————————————
  #                      按键处理
  # ——————————————————————————————————————————————————————————

  def handle_input(self, key):
    """统一按键处理。key: 'UP' | 'DOWN' | 'ENTER' | 'BACK'"""
    if not self.current_page:return

    # —— 编辑模式 ——————————————————————————————————————
    if self.edit_mode:
      if key == 'UP':
        self.edit_temp_val += self.edit_item.step
        if self.edit_temp_val > self.edit_item.max_val:
          self.edit_temp_val = self.edit_item.min_val
      elif key == 'DOWN':
        self.edit_temp_val -= self.edit_item.step
        if self.edit_temp_val < self.edit_item.min_val:
          self.edit_temp_val = self.edit_item.max_val
      elif key == 'ENTER':
        self.edit_item.set_value(self.edit_temp_val)
        if self.edit_item.persistent:
          save_config()
        self.edit_mode = False
        self.edit_item = None
      elif key == 'BACK':
        self.edit_mode = False
        self.edit_item = None
      self._dirty = True
      return

    # —— 导航模式 ——————————————————————————————————————
    items = self.current_page.items
    if not items:return

    if key == 'UP':
      self.focus_index = (self.focus_index - 1) % len(items)
      self._dirty = True
    elif key == 'DOWN':
      self.focus_index = (self.focus_index + 1) % len(items)
      self._dirty = True
    elif key == 'ENTER':
      item = items[self.focus_index]
      if isinstance(item, AdjustItem):
        self.edit_mode = True
        self.edit_item = item
        self.edit_temp_val = item.get_value()
        self._dirty = True
      elif item.action:
        item.action(self, item)
        self._dirty = True    # ★ action 执行后触发重绘
    elif key == 'BACK':
      if self.current_page.on_exit:
        self.current_page.on_exit(self)

  # ——————————————————————————————————————————————————————————
  #                      显示更新
  # ——————————————————————————————————————————————————————————

  def update_display(self):
    """每帧调用：脏检查 → 布局 → 渲染。"""
    page = self.current_page
    if not page:return

    now = ticks_ms()

    # —— 脏检查 ———————————————————————————————————————
    if not self._dirty:
      # 动态页面按 refresh_ms 间隔强制刷新
      if page.refresh_ms:
        if ticks_diff(now, self._last_render_ticks) >= page.refresh_ms:
          self._dirty = True
      if not self._dirty:
        return

    items = page.items
    n = len(items)
    half = (self.max_visible - 1) // 2

    # —— 帧内缓存：预取所有 item 显示文本 ———————————————
    item_texts = []
    for item in items:
      if isinstance(item, AdjustItem):
        item_texts.append(item.text + item.formatter(item.get_value()))
      elif item.get_value:
        item_texts.append(item.get_value())
      else:
        item_texts.append(item.text)

    # —— 构建弧位渲染数据（tuple，非 dict）——————————————
    render_items = []
    for j in range(self.max_visible):
      offset = j - half
      x, y, size = self._arc_slots[j]

      if n > 0:
        idx = (self.focus_index + offset) % n
        item = items[idx]

        if self.edit_mode and item is self.edit_item:
          text = item.text + item.formatter(self.edit_temp_val)
        else:
          text = item_texts[idx]

        focused = (idx == self.focus_index)
      else:
        text = ""
        focused = False

      render_items.append((text, x, y, size, focused))

    # —— 右侧面板数值 ———————————————————————————————————
    value = ""
    if n > 0:
      focus_item = items[self.focus_index]
      if isinstance(focus_item, AdjustItem):
        fmt = focus_item.formatter
        if self.edit_mode and focus_item is self.edit_item:
          value = fmt(self.edit_temp_val)
        else:
          value = fmt(focus_item.get_value())
      elif focus_item.get_value:
        value = focus_item.get_value()
      else:
        value = focus_item.text

    # ★ 先绘制成功再清脏标记：避免 clear 后 OOM 导致永久黑屏
    try:
      self.display(page.name, render_items, value, self.edit_mode)
      self._last_render_ticks = now
      self._dirty = False
    except MemoryError:
      self._dirty = True
      print("[Menu] render OOM, will retry")


# =============================================================================
#                          内部：页面 ID 常量
# =============================================================================

PAGE_MAIN        = 0
PAGE_IMU         = 1
PAGE_HEADING     = 2
PAGE_HEADING_PID = 3
PAGE_TRACKER     = 4
PAGE_TRACKER_PID = 5


# =============================================================================
#                          辅助：跳转 action 工厂
# =============================================================================

def _make_go_action(target_id, focus_index=0):
  def action(menu, item):
    menu.jump_to(target_id, focus_index)
  return action


# =============================================================================
#                          内部：页面工厂函数
# =============================================================================

def _make_main_page():
  """主页 — 导航入口"""
  return MenuPage(
    id=PAGE_MAIN, name="Main Menu",
    items=[
      MenuItem("IMU",     action=_make_go_action(PAGE_IMU, 0)),
      MenuItem("Heading", action=_make_go_action(PAGE_HEADING, 0)),
      MenuItem("Tracker >", action=_make_go_action(PAGE_TRACKER, 0)),
    ],
  )


def _make_imu_page(imu):
  """IMU 状态页（含实时姿态 + 角度阈值）"""
  def _get_ath():  return config["angle_threshold"]
  def _set_ath(v): config["angle_threshold"] = v

  # 实时姿态 (若 IMU 可用)
  if imu is not None:
    _get_yaw   = lambda: f"Yaw:{imu.get_yaw():+.1f}"
    _get_pitch = lambda: f"Pitch:{imu.get_pitch():+.1f}"
    _get_roll  = lambda: f"Roll:{imu.get_roll():+.1f}"

    imu_items = [
      MenuItem("Yaw",   get_value=_get_yaw),
      MenuItem("Pitch", get_value=_get_pitch),
      MenuItem("Roll",  get_value=_get_roll),
    ]
    # 若已标定显示零偏 + 手动重标定，否则显示标定进度
    # ★ get_value 用 lambda 动态读取，避免闭包捕获启动时的冻结值
    imu_items.append(
      MenuItem("Bias",
        get_value=lambda: "G:{:.2f},{:.2f},{:.2f} dps".format(*imu.bias_dps)
          if imu.is_calibrated else "Calibrating...")
    )
    imu_items.append(
      MenuItem("Recal Gyro", action=lambda m, i: imu.recalibrate())
    )
  else:
    imu_items = [
      MenuItem("IMU not connected"),
    ]

  imu_items.append(
    AdjustItem("Thresh:", _get_ath, _set_ath, 1.0, 90.0, 1.0,
               persistent=True, formatter=lambda v: f"{v:.0f} deg")
  )
  imu_items.append(
    MenuItem("[ Back ]", action=_make_go_action(PAGE_MAIN, 0))
  )

  return MenuPage(
    id=PAGE_IMU, name="IMU Status", items=imu_items,
    refresh_ms=200  # 实时数据页，每 200ms 强制刷新
  )


def _make_heading_page(imu, hdg, intents=None, robot=None):
  """航向闭环控制页。有 intents 时只发 Intent，不再直调 hdg。"""
  from app import intent as I
  from app.mode import HDG as ST_HDG, IDLE as ST_IDLE

  _target_yaw = [0.0]

  def _get_hdg_status():
    if not imu or not imu.is_calibrated:
      return "IMU Calibrating..."
    if robot is None or robot.state != ST_HDG:
      if robot is not None and robot.state != ST_IDLE:
        return "FSM:" + robot.state
      return "Status: Idle"
    m = robot.mode
    err = getattr(m, "last_error", 0.0)
    tgt = getattr(m, "target_heading", 0.0)
    sub = getattr(m, "sub_mode", "straight")
    if sub == "straight":
      return "Straight | Err:{:+.1f} | Spd:{:.0f}%".format(
        err, getattr(m, "forward_speed", 0.0))
    return "Lock -> {:+.0f} | Err:{:+.1f}".format(tgt, err)

  def _get_target_yaw(): return _target_yaw[0]
  def _set_target_yaw(v): _target_yaw[0] = v
  def _get_speed(): return config["target_speed"]
  def _set_speed(v): config["target_speed"] = v

  def _go_straight(m, i):
    if intents is not None:
      intents.post(I.GO_STRAIGHT)

  def _lock_yaw(m, i):
    if intents is not None:
      intents.post(I.LOCK_YAW, _target_yaw[0])

  def _lock_cur(m, i):
    if intents is not None:
      intents.post(I.LOCK_YAW)

  def _stop(m, i):
    if intents is not None:
      intents.post(I.STOP)

  return MenuPage(
    id=PAGE_HEADING, name="Heading Control",
    items=[
      MenuItem("Status", get_value=_get_hdg_status),
      AdjustItem("Speed:", _get_speed, _set_speed,
                 0.0, 100.0, 5.0, persistent=True,
                 formatter=lambda v: "{:.0f}%".format(v)),
      MenuItem("Go Straight", action=_go_straight),
      AdjustItem("Target:", _get_target_yaw, _set_target_yaw,
                 -180.0, 180.0, 5.0, persistent=False,
                 formatter=lambda v: "{:+.0f} deg".format(v)),
      MenuItem("Lock Yaw", action=_lock_yaw),
      MenuItem("Lock Current", action=_lock_cur),
      MenuItem("STOP", action=_stop),
      MenuItem("PID Tune >", action=_make_go_action(PAGE_HEADING_PID, 0)),
      MenuItem("[ Back ]", action=_make_go_action(PAGE_MAIN, 1)),
    ],
    refresh_ms=200,
  )


def _make_heading_pid_page(hdg):
  """航向 PID 调参页（引用模式 → update_pid_gains 为 no-op）"""
  def _get_hkp():  return config["heading_kp"]
  def _set_hkp(v): config["heading_kp"] = v
  def _get_hki():  return config["heading_ki"]
  def _set_hki(v): config["heading_ki"] = v
  def _get_hkd():  return config["heading_kd"]
  def _set_hkd(v): config["heading_kd"] = v
  def _get_hmax(): return config["heading_max_correction"]
  def _set_hmax(v): config["heading_max_correction"] = v
  def _get_hdb():  return config["heading_deadband"]
  def _set_hdb(v): config["heading_deadband"] = v

  return MenuPage(
    id=PAGE_HEADING_PID, name="Heading PID",
    items=[
      AdjustItem("Kp:", _get_hkp, _set_hkp,
                 0.0, 20.0, 0.1, persistent=True,
                 formatter=lambda v: "{:.2f}".format(v)),
      AdjustItem("Ki:", _get_hki, _set_hki,
                 0.0, 5.0, 0.01, persistent=True,
                 formatter=lambda v: "{:.3f}".format(v)),
      AdjustItem("Kd:", _get_hkd, _set_hkd,
                 0.0, 5.0, 0.01, persistent=True,
                 formatter=lambda v: "{:.3f}".format(v)),
      AdjustItem("MaxCorr:", _get_hmax, _set_hmax,
                 5.0, 100.0, 5.0, persistent=True,
                 formatter=lambda v: "{:.0f}%".format(v)),
      AdjustItem("Deadband:", _get_hdb, _set_hdb,
                 0.0, 10.0, 0.5, persistent=True,
                 formatter=lambda v: "{:.1f} deg".format(v)),
      MenuItem("[ Back ]", action=_make_go_action(PAGE_HEADING, 7)),
    ],
  )


def _make_tracker_page(tracker, camera, intents=None, robot=None):
  """视觉跟踪主页面。有 intents 时只发 Intent。"""
  from app import intent as I
  from app.mode import IDLE, SEARCH, TRACK, COMPLETE, FAULT

  CLASS_NAMES = {7: "Any", 0: "Sandbag", 1: "Netball", 2: "Bear"}
  _STATE_MAP = {
    IDLE: "IDLE", SEARCH: "SEARCHING", TRACK: "TRACKING",
    COMPLETE: "COMPLETE", FAULT: "FAULT", "HDG": "IDLE",
  }

  def _get_status():
    if camera is None:
      return "Camera: N/A"
    if getattr(camera, "failed", False):
      return "Camera: Self-Test FAILED"
    if not camera.is_ready:
      return "Camera: Disconnected"
    n = len(camera.detections)
    if robot is None:
      return "{} targets".format(n)
    st = _STATE_MAP.get(robot.state, robot.state)
    if st == "IDLE":
      return "Idle | {} targets".format(n)
    if st == "SEARCHING":
      return "Searching... | {} targets".format(n)
    if st == "TRACKING":
      info = robot.target_info if robot.target_info else "--"
      return "TRACK | {} | {} targets".format(info, n)
    if st == "COMPLETE":
      return "COMPLETE [BACK to return]"
    if st == "FAULT":
      return "FAULT — Reconnect"
    return st

  def _get_tgt_cls():  return config["trk_target_class"]
  def _set_tgt_cls(v): config["trk_target_class"] = int(v)
  def _fmt_cls(v):
    return CLASS_NAMES.get(int(v), str(int(v)))

  def _get_min_conf():  return config["trk_min_confidence"]
  def _set_min_conf(v): config["trk_min_confidence"] = int(v)
  def _get_app_spd():  return config["trk_approach_speed"]
  def _set_app_spd(v): config["trk_approach_speed"] = v
  def _get_srch_spd(): return config["trk_search_speed"]
  def _set_srch_spd(v): config["trk_search_speed"] = v
  def _get_stop_pct(): return config["trk_stop_bottom_pct"]
  def _set_stop_pct(v): config["trk_stop_bottom_pct"] = v
  def _get_rev_ang():  return config["trk_reverse_angle"]
  def _set_rev_ang(v): config["trk_reverse_angle"] = v

  def _action_start(m, i):
    if intents is not None:
      intents.post(I.START_TRACK)

  def _action_stop(m, i):
    if intents is not None:
      intents.post(I.STOP)

  def _action_reconnect(m, i):
    if intents is not None:
      intents.post(I.RECONNECT)

  return MenuPage(
    id=PAGE_TRACKER, name="Object Tracker",
    items=[
      MenuItem("Status", get_value=_get_status),
      MenuItem("Start Track", action=_action_start),
      MenuItem("Stop", action=_action_stop),
      MenuItem("Reconnect", action=_action_reconnect),
      AdjustItem("Class:", _get_tgt_cls, _set_tgt_cls,
                 0, 7, 1, persistent=True, formatter=_fmt_cls),
      AdjustItem("Confidence:", _get_min_conf, _set_min_conf,
                 0, 31, 1, persistent=True,
                 formatter=lambda v: "{:.0f}/31".format(v)),
      AdjustItem("ApproachSpd:", _get_app_spd, _set_app_spd,
                 5.0, 100.0, 5.0, persistent=True,
                 formatter=lambda v: "{:.0f}%".format(v)),
      AdjustItem("SearchSpd:", _get_srch_spd, _set_srch_spd,
                 5.0, 50.0, 5.0, persistent=True,
                 formatter=lambda v: "{:.0f}%".format(v)),
      AdjustItem("StopBottom:", _get_stop_pct, _set_stop_pct,
                 70.0, 99.0, 1.0, persistent=True,
                 formatter=lambda v: "{:.0f}%".format(v)),
      AdjustItem("RevAngle:", _get_rev_ang, _set_rev_ang,
                 10.0, 90.0, 5.0, persistent=True,
                 formatter=lambda v: "{:.0f} deg".format(v)),
      MenuItem("PID Tune >", action=_make_go_action(PAGE_TRACKER_PID, 0)),
      MenuItem("[ Back ]", action=_make_go_action(PAGE_MAIN, 2)),
    ],
    refresh_ms=200,
  )


def _make_tracker_pid_page(tracker):
  """视觉跟踪 PID 调参页（引用模式）"""
  def _get_kp():  return config["trk_bearing_kp"]
  def _set_kp(v): config["trk_bearing_kp"] = v
  def _get_ki():  return config["trk_bearing_ki"]
  def _set_ki(v): config["trk_bearing_ki"] = v
  def _get_kd():  return config["trk_bearing_kd"]
  def _set_kd(v): config["trk_bearing_kd"] = v
  def _get_max(): return config["trk_bearing_max"]
  def _set_max(v): config["trk_bearing_max"] = v
  def _get_db():  return config["trk_bearing_db"]
  def _set_db(v): config["trk_bearing_db"] = v

  return MenuPage(
    id=PAGE_TRACKER_PID, name="Tracker PID",
    items=[
      AdjustItem("Kp:", _get_kp, _set_kp,
                 0.0, 10.0, 0.1, persistent=True,
                 formatter=lambda v: "{:.2f}".format(v)),
      AdjustItem("Ki:", _get_ki, _set_ki,
                 0.0, 2.0, 0.01, persistent=True,
                 formatter=lambda v: "{:.3f}".format(v)),
      AdjustItem("Kd:", _get_kd, _set_kd,
                 0.0, 2.0, 0.01, persistent=True,
                 formatter=lambda v: "{:.3f}".format(v)),
      AdjustItem("MaxRotation:", _get_max, _set_max,
                 5.0, 100.0, 5.0, persistent=True,
                 formatter=lambda v: "{:.0f}%".format(v)),
      AdjustItem("Deadband:", _get_db, _set_db,
                 0.0, 0.3, 0.01, persistent=True,
                 formatter=lambda v: "{:.2f}".format(v)),
      MenuItem("[ Back ]", action=_make_go_action(PAGE_TRACKER, 9)),
    ],
    refresh_ms=200,
  )


# =============================================================================
#                          内部：页面注册
# =============================================================================

def _register_pages(imu=None, hdg=None, tracker=None, camera=None,
                    intents=None, robot=None):
  """注册所有页面到全局注册表。由 MenuInit 调用。"""
  _register(_make_main_page())
  _register(_make_imu_page(imu))

  if hdg is not None or robot is not None:
    _register(_make_heading_page(imu, hdg, intents=intents, robot=robot))
    _register(_make_heading_pid_page(hdg))

  if tracker is not None or robot is not None:
    _register(_make_tracker_page(tracker, camera, intents=intents, robot=robot))
    _register(_make_tracker_pid_page(tracker))


# =============================================================================
#                         公开 API
# =============================================================================

def MenuInit(W=320, H=200, Cx=None,
             csPin='B29', rstPin='B31', dcPin='B5', blkPin='C21',
             spiIndex=2, baudrate=60000000,
             step_angle=18, max_visible=5, base_size=16,
             imu=None, hdg=None, tracker=None, camera=None,
             intents=None, robot=None,
             _lcd=None, _lcd_drv=None):
  """
  初始化菜单系统：屏幕 → 显示驱动 → 加载配置 → 创建 Menu → 注册页面 → 进入主页。

  参数：
    W, H             — 屏幕宽高 (默认 320x200)
    Cx               — 圆弧圆心 x 偏移 (默认自动计算，适配黄金分割布局)
    csPin, rstPin, dcPin, blkPin — IPS200 引脚
    spiIndex          — SPI 索引
    baudrate          — SPI 波特率
    step_angle        — 相邻项角距 (度)
    max_visible       — 最大可见项数 (须为奇数)
    base_size         — 最大字号 (相对值)
    imu               — ImuSensor 实例 (用于实时数据页)
    hdg               — HeadingController 实例 (用于航向控制页)

  返回：
    Menu 实例 — 调用 menu.handle_input(key) 和 menu.update_display()
  """
  # 1. 初始化 IPS200 屏幕（若外部已创建则复用）
  if _lcd is not None:
    lcd = _lcd
    lcd_drv = _lcd_drv
  else:
    from machine import Pin
    from display import LCD_Drv, LCD
    cs = Pin(csPin, Pin.OUT, value=True); cs.high(); cs.low()
    rst = Pin(rstPin, Pin.OUT, value=True)
    dc  = Pin(dcPin,  Pin.OUT, value=True)
    blk = Pin(blkPin, Pin.OUT, value=True)

    lcd_drv = LCD_Drv(SPI_INDEX=spiIndex, BAUDRATE=baudrate,
                      DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
    lcd = LCD(lcd_drv)
    lcd.mode(1)  # 横屏
    lcd.color(0xFFFF, 0x0000)
    lcd.clear(0x0000)

  # 2. 创建显示驱动
  driver = DisplayDriver(lcd, W=W, H=H)

  # 3. 创建菜单（config 已由 main load）
  menu = Menu(driver.render, W=W, H=H, Cx=Cx,
              step_angle=step_angle, max_visible=max_visible,
              base_size=base_size)

  # 4. 注册页面并进入主页（立刻画一帧，避免之后 RAM 更紧时永久黑屏）
  _register_pages(imu, hdg, tracker, camera, intents=intents, robot=robot)
  menu.goto(get_page(PAGE_MAIN))
  menu.update_display()

  return menu


def MenuHelp():
  return "Ring Menu: UP/DOWN/ENTER/BACK"
