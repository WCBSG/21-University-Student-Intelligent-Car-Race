import math
from time import ticks_ms, ticks_diff
from machine import Pin
from display import LCD_Drv, LCD
from config import config, load_config, save_config

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
        self.edit_temp_val = min(
          self.edit_item.max_val,
          self.edit_temp_val + self.edit_item.step
        )
      elif key == 'DOWN':
        self.edit_temp_val = max(
          self.edit_item.min_val,
          self.edit_temp_val - self.edit_item.step
        )
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

    self._last_render_ticks = now
    self._dirty = False

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

    # 调用 DisplayDriver.render(title, items, value, edit_mode)
    self.display(page.name, render_items, value, self.edit_mode)


# =============================================================================
#                          内部：页面 ID 常量
# =============================================================================

PAGE_MAIN        = 0
PAGE_IMU         = 1
PAGE_ABOUT       = 2
PAGE_HEADING     = 3
PAGE_HEADING_PID = 4


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
      MenuItem("About",   action=_make_go_action(PAGE_ABOUT, 0)),
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
    if imu.is_calibrated:
      bx, by, bz = imu.bias_dps
      imu_items.append(
        MenuItem("Bias", get_value=lambda: f"G:{bx:.2f},{by:.2f},{bz:.2f} dps")
      )
      imu_items.append(
        MenuItem("Recal Gyro", action=lambda m, i: imu.recalibrate())
      )
    else:
      imu_items.append(
        MenuItem("Calibrating...")
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


def _make_about_page():
  """About / 引脚状态页"""
  # 注意：C8/C9/C14/C15 与 KEY_HANDLER 共用，Pin 初始化可能冲突
  try:
    _pin_c8  = Pin('C8',  Pin.IN, pull=Pin.PULL_UP_47K)
    _pin_c9  = Pin('C9',  Pin.IN, pull=Pin.PULL_UP_47K)
    _pin_c14 = Pin('C14', Pin.IN, pull=Pin.PULL_UP_47K)
    _pin_c15 = Pin('C15', Pin.IN, pull=Pin.PULL_UP_47K)

    _get_c8  = lambda: "C8:H"  if _pin_c8.value()  else "C8:L"
    _get_c9  = lambda: "C9:H"  if _pin_c9.value()  else "C9:L"
    _get_c14 = lambda: "C14:H" if _pin_c14.value() else "C14:L"
    _get_c15 = lambda: "C15:H" if _pin_c15.value() else "C15:L"
  except:
    _get_c8  = lambda: "C8:--"
    _get_c9  = lambda: "C9:--"
    _get_c14 = lambda: "C14:--"
    _get_c15 = lambda: "C15:--"

  return MenuPage(
    id=PAGE_ABOUT, name="About",
    items=[
      MenuItem("C8",  get_value=_get_c8),
      MenuItem("C9",  get_value=_get_c9),
      MenuItem("C14", get_value=_get_c14),
      MenuItem("C15", get_value=_get_c15),
      MenuItem("[ Back ]", action=_make_go_action(PAGE_MAIN, 2)),
    ],
  )


def _make_heading_page(imu, hdg):
  """航向闭环控制页"""
  # 目标航向（闭包可变捕获，MicroPython 无 nonlocal，用 list）
  _target_yaw = [0.0]

  # 状态行
  def _get_hdg_status():
    if not imu or not imu.is_calibrated:
      return "IMU Calibrating..."
    mode = hdg.mode
    if mode == 'idle':
      return "Status: Idle"
    err = hdg.heading_error
    tgt = hdg.target_heading
    if mode == 'straight':
      return "Straight | Err:{:+.1f} | Spd:{:.0f}%".format(err, hdg.forward_speed)
    elif mode == 'lock':
      return "Lock -> {:+.0f} | Err:{:+.1f}".format(tgt, err)
    return "Status: " + mode

  # 目标航向 get/set (闭包捕获 _target_yaw)
  def _get_target_yaw(): return _target_yaw[0]
  def _set_target_yaw(v): _target_yaw[0] = v

  def _get_speed(): return config["target_speed"]
  def _set_speed(v): config["target_speed"] = v

  return MenuPage(
    id=PAGE_HEADING, name="Heading Control",
    items=[
      MenuItem("Status", get_value=_get_hdg_status),

      AdjustItem("Speed:", _get_speed, _set_speed,
                 0.0, 100.0, 5.0, persistent=True,
                 formatter=lambda v: "{:.0f}%".format(v)),

      MenuItem("Go Straight", action=lambda m, i: hdg.mode_straight()),

      AdjustItem("Target:", _get_target_yaw, _set_target_yaw,
                 -180.0, 180.0, 5.0, persistent=False,
                 formatter=lambda v: "{:+.0f} deg".format(v)),

      MenuItem("Lock Yaw", action=lambda m, i: hdg.mode_lock(_target_yaw[0])),

      MenuItem("Lock Current", action=lambda m, i: hdg.mode_lock()),

      MenuItem("STOP", action=lambda m, i: hdg.mode_idle()),

      MenuItem("PID Tune >", action=_make_go_action(PAGE_HEADING_PID, 0)),

      MenuItem("[ Back ]", action=_make_go_action(PAGE_MAIN, 1)),
    ],
  )


def _make_heading_pid_page(hdg):
  """航向 PID 调参页"""
  def _get_hkp():  return config["heading_kp"]
  def _set_hkp(v): config["heading_kp"] = v; hdg.update_pid_gains()
  def _get_hki():  return config["heading_ki"]
  def _set_hki(v): config["heading_ki"] = v; hdg.update_pid_gains()
  def _get_hkd():  return config["heading_kd"]
  def _set_hkd(v): config["heading_kd"] = v; hdg.update_pid_gains()
  def _get_hmax(): return config["heading_max_correction"]
  def _set_hmax(v): config["heading_max_correction"] = v; hdg.update_pid_gains()
  def _get_hdb():  return config["heading_deadband"]
  def _set_hdb(v): config["heading_deadband"] = v; hdg.update_pid_gains()

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


# =============================================================================
#                          内部：页面注册
# =============================================================================

def _register_pages(imu=None, hdg=None):
  """注册所有页面到全局注册表。由 MenuInit 调用。"""
  _register(_make_main_page())
  _register(_make_imu_page(imu))
  _register(_make_about_page())

  if hdg is not None:
    _register(_make_heading_page(imu, hdg))
    _register(_make_heading_pid_page(hdg))


# =============================================================================
#                         公开 API
# =============================================================================

def MenuInit(W=320, H=200, Cx=None,
             csPin='B29', rstPin='B31', dcPin='B5', blkPin='C21',
             spiIndex=2, baudrate=60000000,
             step_angle=18, max_visible=5, base_size=16,
             imu=None, hdg=None):
  """
  初始化菜单系统：屏幕 → 显示驱动 → 加载配置 → 创建 Menu → 注册页面 → 进入主页。

  参数：
    W, H             — 屏幕宽高 (默认 320×200)
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
  # 1. 初始化 IPS200 屏幕
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

  # 3. 加载持久化配置
  load_config()

  # 4. 创建菜单
  menu = Menu(driver.render, W=W, H=H, Cx=Cx,
              step_angle=step_angle, max_visible=max_visible,
              base_size=base_size)

  # 5. 注册页面并进入主页
  _register_pages(imu, hdg)
  menu.goto(get_page(PAGE_MAIN))

  return menu


def MenuHelp():
  """返回菜单系统帮助信息。"""
  return """
=== Ring Menu System ===

Public API:
  MenuInit(**kwargs) -> Menu
  MenuHelp() -> str

Menu Methods:
  goto(page, focus_index=0)   — Switch to a page
  jump_to(page_id, focus=0)   — Jump by page ID
  handle_input(key)           — Process key: UP/DOWN/ENTER/BACK
  update_display()            — Render arc layout + info panel

Key Mapping:
  KEY1 (C8) = UP       — Previous item / Increase value
  KEY2 (C9) = DOWN     — Next item / Decrease value
  KEY3 (C14)= ENTER    — Select / Confirm edit
  KEY4 (C15)= BACK     — Go back / Cancel edit

Pages:
  0: Main Menu     — IMU / Heading / About
  1: IMU Status    — Yaw/Pitch/Roll (live) + Thresh
  2: About         — Live pin states (C8/C9/C14/C15)
  3: Heading Ctrl  — Go Straight / Lock Yaw / PID tuning
  4: Heading PID   — Kp/Ki/Kd / MaxCorr / Deadband

Layout:
  Left 38.2%  — Arc menu (5 visible items, focus centered)
  Right 61.8% — Title (top) + Value (middle)

Config:
  Params stored in /flash/config.json (auto-created)
  AdjustItems with persistent=True auto-save on ENTER
"""
