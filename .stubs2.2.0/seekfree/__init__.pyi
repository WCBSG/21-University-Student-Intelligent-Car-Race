"""
seekfree module for RT1021-MicroPython v2.2.0
Seekfree vendor-specific drivers for sensors, motors, display, and communication.
"""

from typing import Any, Callable, List, Optional, Tuple, Union
from array import array
from machine import Pin
from smartcar import ticker


# ---------------------------------------------------------------------------
# MOTOR_CONTROLLER -- DC motor driver (DRV8701 / HIP4082)
# ---------------------------------------------------------------------------

class MOTOR_CONTROLLER:
    """DC motor controller (PWM+DIR or PWM+PWM mode)."""

    # PWM+DIR mode (DRV8701) pin pairs
    PWM_C30_DIR_C31: int = ...
    PWM_C28_DIR_C29: int = ...
    PWM_D4_DIR_D5: int = ...
    PWM_D6_DIR_D7: int = ...
    PWM_C24_DIR_C26: int = ...
    PWM_C25_DIR_C27: int = ...

    # PWM+PWM mode (HIP4082) pin pairs
    PWM_C30_PWM_C31: int = ...
    PWM_C28_PWM_C29: int = ...
    PWM_D4_PWM_D5: int = ...
    PWM_D6_PWM_D7: int = ...
    PWM_C24_PWM_C26: int = ...
    PWM_C25_PWM_C27: int = ...

    def __init__(self, index: int, freq: int, duty: int = 0, invert: bool = False) -> None:
        """
        Construct a motor controller.

        :param index: Pin pair constant (e.g. MOTOR_CONTROLLER.PWM_C30_DIR_C31)
        :param freq: PWM frequency [1, 100000]
        :param duty: Initial duty [-10000, 10000], sign depends on invert
        :param invert: Invert direction
        """
        ...

    def duty(self, duty: Optional[int] = None) -> int:
        """
        Get or set motor duty.

        :param duty: New duty [-10000, 10000]
        :return: Current duty
        """
        ...

    def info(self) -> None:
        """Print object information."""
        ...

    @staticmethod
    def help() -> None:
        """Print usage help."""
        ...


# ---------------------------------------------------------------------------
# BLDC_CONTROLLER -- Brushless DC motor (servo-style PWM)
# ---------------------------------------------------------------------------

class BLDC_CONTROLLER:
    """Brushless DC motor controller (50-300Hz servo PWM)."""

    PWM_C25: int = ...
    PWM_C27: int = ...
    PWM_B26: int = ...
    PWM_B27: int = ...

    def __init__(self, index: int, freq: int = 50, highlevel_us: int = 1000) -> None:
        """
        Construct a BLDC controller.

        :param index: Pin constant (e.g. BLDC_CONTROLLER.PWM_C25)
        :param freq: PWM frequency [50, 300]
        :param highlevel_us: High-level pulse width in us [1000, 2000]
        """
        ...

    def highlevel_us(self, us: Optional[int] = None) -> int:
        """
        Get or set high-level pulse width.

        :param us: Pulse width in us [1000, 2000]
        :return: Current pulse width in us
        """
        ...

    def info(self) -> None:
        """Print object information."""
        ...

    @staticmethod
    def help() -> None:
        """Print usage help."""
        ...


# ---------------------------------------------------------------------------
# KEY_HANDLER -- Button handler
# ---------------------------------------------------------------------------

class KEY_HANDLER:
    """Button handler with short/long press detection."""

    def __init__(self, period: int = 10) -> None:
        """
        Construct a key handler.

        :param period: Scan period (ticker unit count, default 10)
        """
        ...

    def capture(self) -> None:
        """Add a capture request (called by ticker or manually)."""
        ...

    def get(self) -> List[int]:
        """
        Get key states.

        :return: List of key states [0=none, 1=short press, 2=long press] for keys 1-4
        """
        ...

    def read(self) -> List[int]:
        """Immediate capture and read key states."""
        ...

    def clear(self, index: Optional[int] = None) -> None:
        """
        Clear key state(s).

        :param index: Key index [1, 4] or None to clear all
        """
        ...

    def get_period(self) -> int:
        """Get current scan period."""
        ...

    def info(self) -> None:
        """Print object information."""
        ...

    @staticmethod
    def help() -> None:
        """Print usage help."""
        ...


# ---------------------------------------------------------------------------
# IMU660RX -- 6-axis IMU (LSM6DSO)
# ---------------------------------------------------------------------------

class IMU660RX:
    """6-axis IMU (accelerometer + gyroscope)."""

    # Module type constants
    TYPE_AUTO: int = ...
    TYPE_RA: int = ...
    TYPE_RB: int = ...
    TYPE_RC: int = ...

    # Hardware quaternion rate constants
    RATE_15HZ: int = ...
    RATE_30HZ: int = ...
    RATE_60HZ: int = ...
    RATE_120HZ: int = ...
    RATE_240HZ: int = ...
    RATE_480HZ: int = ...
    RATE_DISABLE: int = ...

    def __init__(self, capture_div: int = 1, imu_type: int = TYPE_AUTO, quar_rate: int = RATE_DISABLE) -> None:
        """
        Construct an IMU object.

        :param capture_div: Capture divider (n ticker ticks per capture)
        :param imu_type: Module type (TYPE_AUTO, TYPE_RA, TYPE_RB, TYPE_RC)
        :param quar_rate: Hardware quaternion rate (RATE_15HZ...RATE_480HZ, RATE_DISABLE)
                          When enabled, INT2 pin must be connected for hardware interrupt.
        """
        ...

    def capture(self) -> None:
        """Add a capture request (called by ticker or manually)."""
        ...

    def get(self) -> List[int]:
        """
        Get raw sensor data buffer (linked list - auto-updates on capture).

        :return: [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
        """
        ...

    def get_euler(self) -> List[float]:
        """
        Get Euler angle data buffer (requires TYPE_RC with hardware quaternion enabled).

        :return: [roll, pitch, yaw] in degrees
        """
        ...

    def get_quarternion(self) -> List[float]:
        """
        Get quaternion data buffer (requires TYPE_RC with hardware quaternion enabled).

        :return: [x, y, z, w]
        """
        ...

    def read(self) -> List[int]:
        """Immediate capture and read raw sensor data."""
        ...

    def get_capture_div(self) -> int:
        """Get current capture divider."""
        ...

    def info(self) -> None:
        """Print object information."""
        ...

    @staticmethod
    def help() -> None:
        """Print usage help, including supported rates and ranges."""
        ...


# ---------------------------------------------------------------------------
# IMU963RX -- 9-axis IMU
# ---------------------------------------------------------------------------

class IMU963RX:
    """9-axis IMU (accelerometer + gyroscope + magnetometer)."""

    # Module type constants
    TYPE_AUTO: int = ...
    TYPE_RA: int = ...
    TYPE_RB: int = ...
    TYPE_RC: int = ...

    # Hardware quaternion rate constants
    RATE_15HZ: int = ...
    RATE_30HZ: int = ...
    RATE_60HZ: int = ...
    RATE_120HZ: int = ...
    RATE_240HZ: int = ...
    RATE_480HZ: int = ...
    RATE_DISABLE: int = ...

    def __init__(self, capture_div: int = 1, imu_type: int = TYPE_AUTO, quar_rate: int = RATE_DISABLE) -> None:
        """
        Construct an IMU object.

        :param capture_div: Capture divider (n ticker ticks per capture)
        :param imu_type: Module type (TYPE_AUTO, TYPE_RA, TYPE_RB, TYPE_RC)
        :param quar_rate: Hardware quaternion rate (RATE_15HZ...RATE_480HZ, RATE_DISABLE)
        """
        ...

    def capture(self) -> None:
        """Add a capture request."""
        ...

    def get(self) -> List[int]:
        """
        Get raw sensor data buffer.

        :return: [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z]
        """
        ...

    def get_euler(self) -> List[float]:
        """
        Get Euler angle data buffer.

        :return: [roll, pitch, yaw] in degrees
        """
        ...

    def get_quarternion(self) -> List[float]:
        """
        Get quaternion data buffer.

        :return: [x, y, z, w]
        """
        ...

    def read(self) -> List[int]:
        """Immediate capture and read raw sensor data."""
        ...

    def get_capture_div(self) -> int:
        """Get current capture divider."""
        ...

    def info(self) -> None:
        """Print object information."""
        ...

    @classmethod
    def help(cls) -> None:
        """Print usage help."""
        ...


# ---------------------------------------------------------------------------
# DL1X -- ToF distance sensor
# ---------------------------------------------------------------------------

class DL1X:
    """Time-of-Flight distance sensor (e.g. DL1A, DL1B)."""

    def __init__(self, capture_div: int = 1) -> None:
        """
        Construct a ToF sensor object.

        :param capture_div: Capture divider (n ticker ticks per capture)
        """
        ...

    def capture(self) -> None:
        """Add a capture request."""
        ...

    def get(self) -> int:
        """
        Get distance.

        :return: Distance in mm
        """
        ...

    def read(self) -> int:
        """Immediate capture and read distance in mm."""
        ...

    def info(self) -> None:
        """Print object information."""
        ...

    @staticmethod
    def help() -> None:
        """Print usage help."""
        ...


# ---------------------------------------------------------------------------
# TSL1401 -- Linear CCD sensor
# ---------------------------------------------------------------------------

class TSL1401:
    """Linear CCD sensor (128 pixels, with up to 4 CCD modules)."""

    # Resolution constants
    RES_8BIT: int = ...
    RES_10BIT: int = ...
    RES_12BIT: int = ...

    def __init__(self, capture_div: int = 1) -> None:
        """
        Construct a CCD object.

        :param capture_div: Capture divider (n ticker ticks per capture). Note that
                           TSL1401 exposure time = capture_div * ticker period.
        """
        ...

    def set_resolution(self, resolution: int) -> None:
        """
        Set ADC resolution for CCD.

        :param resolution: RES_8BIT, RES_10BIT, or RES_12BIT
        """
        ...

    def capture(self) -> None:
        """Add a capture request."""
        ...

    def get(self, index: int = 0) -> 'array':
        """
        Get CCD pixel data for the specified channel.

        :param index: CCD channel index [0, 3]
        :return: Array of pixel values
        """
        ...

    def read(self) -> 'array':
        """Immediate capture and read CCD data."""
        ...

    def info(self) -> None:
        """Print object information."""
        ...

    @staticmethod
    def help() -> None:
        """Print usage help."""
        ...


# ---------------------------------------------------------------------------
# WIRELESS_UART -- Wireless serial module
# ---------------------------------------------------------------------------

class WIRELESS_UART:
    """Wireless UART communication module."""

    # CCD buffer indices for send_ccd_image
    CCD1_BUFFER_INDEX: int = ...
    CCD2_BUFFER_INDEX: int = ...
    CCD3_BUFFER_INDEX: int = ...
    CCD4_BUFFER_INDEX: int = ...
    CCD1_2_BUFFER_INDEX: int = ...
    CCD3_4_BUFFER_INDEX: int = ...

    def __init__(self, baudrate: int = 460800) -> None:
        """
        Construct a wireless UART object.

        :param baudrate: Communication baud rate (default 460800)
        """
        ...

    def send_str(self, s: str) -> None:
        """Send a string."""
        ...

    def send_bytearray(self, array: 'array', length: int) -> None:
        """
        Send a byte array.

        :param array: Byte-type array ('b' typecode)
        :param length: Number of bytes to send
        """
        ...

    def receive_bytearray(self, array: 'array', length: int) -> int:
        """
        Receive data into a byte array.

        :param array: Byte-type array ('b' typecode)
        :param length: Maximum number of bytes to receive
        :return: Number of bytes actually received
        """
        ...

    def send_oscilloscope(self, d1: float, d2: float = ..., d3: float = ..., d4: float = ..., d5: float = ..., d6: float = ..., d7: float = ..., d8: float = ...) -> None:
        """
        Send virtual oscilloscope data to Seekfree Assistant (1-8 channels).

        :param dx: Float data for each channel
        """
        ...

    def send_ccd_image(self, index: int) -> None:
        """
        Upload CCD image data to Seekfree Assistant.

        :param index: CCD_BUFFER_INDEX constant specifying which CCD channel(s)
        """
        ...

    def data_analysis(self) -> List[int]:
        """
        Parse tuning parameter data from Seekfree Assistant.

        :return: List of 8 flags indicating which channels have new data
        """
        ...

    def get_data(self, index: int = ...) -> List[float]:
        """
        Get tuning parameter values from Seekfree Assistant.

        :param index: Channel index or all channels if omitted
        :return: List of 8 float values
        """
        ...

    def info(self) -> None:
        """Print object information."""
        ...

    @staticmethod
    def help() -> None:
        """Print usage help."""
        ...


# ---------------------------------------------------------------------------
# WIFI_SPI -- WiFi SPI module
# ---------------------------------------------------------------------------

class WIFI_SPI:
    """WiFi SPI communication module."""

    # Connection type constants
    TCP_CONNECT: int = ...
    UDP_CONNECT: int = ...

    # CCD buffer indices for send_ccd_image
    CCD1_BUFFER_INDEX: int = ...
    CCD2_BUFFER_INDEX: int = ...
    CCD3_BUFFER_INDEX: int = ...
    CCD4_BUFFER_INDEX: int = ...
    CCD1_2_BUFFER_INDEX: int = ...
    CCD3_4_BUFFER_INDEX: int = ...

    def __init__(self, wifi_ssid: str, pass_word: str, connect_type: int, ip_addr: str, connect_port: str) -> None:
        """
        Construct a WiFi SPI object.
        Note: RT1021_100P_2P54 does NOT support WIFI_SPI.
        Initialization takes up to several minutes.

        :param wifi_ssid: WiFi hotspot name
        :param pass_word: WiFi hotspot password
        :param connect_type: TCP_CONNECT or UDP_CONNECT
        :param ip_addr: Target IP address
        :param connect_port: Target port (as string)
        """
        ...

    def send_str(self, s: str) -> None:
        """Send a string."""
        ...

    def send_bytearray(self, array: 'array', length: int) -> None:
        """
        Send a byte array.

        :param array: Byte-type array ('b' typecode)
        :param length: Number of bytes to send
        """
        ...

    def receive_bytearray(self, array: 'array', length: int) -> int:
        """
        Receive data into a byte array.

        :param array: Byte-type array ('b' typecode)
        :param length: Maximum number of bytes to receive
        :return: Number of bytes actually received
        """
        ...

    def send_oscilloscope(self, d1: float, d2: float = ..., d3: float = ..., d4: float = ..., d5: float = ..., d6: float = ..., d7: float = ..., d8: float = ...) -> None:
        """
        Send virtual oscilloscope data to Seekfree Assistant (1-8 channels).

        :param dx: Float data for each channel
        """
        ...

    def send_ccd_image(self, index: int) -> None:
        """
        Upload CCD image data to Seekfree Assistant.

        :param index: CCD_BUFFER_INDEX constant
        """
        ...

    def data_analysis(self) -> List[int]:
        """
        Parse tuning parameter data from Seekfree Assistant.

        :return: List of 8 flags indicating which channels have new data
        """
        ...

    def get_data(self, index: int = ...) -> List[float]:
        """
        Get tuning parameter values from Seekfree Assistant.

        :param index: Channel index or all channels if omitted
        :return: List of 8 float values
        """
        ...

    def info(self) -> None:
        """Print object information."""
        ...

    @staticmethod
    def help() -> None:
        """Print usage help."""
        ...


# ---------------------------------------------------------------------------
# IPS200PRO -- IPS200PRO display with widget library
# ---------------------------------------------------------------------------

class IPS200PRO:
    """
    IPS200PRO display module with rich widget library.

    Widgets are created via methods on the IPS200PRO instance and managed by
    integer IDs. Universal methods (set_font, set_color, set_position,
    set_hidden, set_parent) work on all widget types.
    """

    # -- Title position --
    TITLE_LEFT: int = ...
    TITLE_RIGHT: int = ...
    TITLE_TOP: int = ...
    TITLE_BOTTOM: int = ...

    # -- Page animation --
    PAGE_ANIM_OFF: int = ...
    PAGE_ANIM_ON: int = ...

    # -- Font sizes (only 16, 20, 24 support Chinese) --
    FONT_SIZE_12: int = ...
    FONT_SIZE_14: int = ...
    FONT_SIZE_16: int = ...
    FONT_SIZE_18: int = ...
    FONT_SIZE_20: int = ...
    FONT_SIZE_22: int = ...
    FONT_SIZE_24: int = ...
    FONT_SIZE_26: int = ...
    FONT_SIZE_28: int = ...
    FONT_SIZE_30: int = ...
    FONT_SIZE_32: int = ...
    FONT_SIZE_34: int = ...
    FONT_SIZE_36: int = ...
    FONT_SIZE_40: int = ...

    # -- Color target types --
    COLOR_FOREGROUND: int = ...
    COLOR_BACKGROUND: int = ...
    COLOR_BORDER: int = ...
    COLOR_PAGE_SELECTED_TEXT: int = ...
    COLOR_PAGE_SELECTED_BG: int = ...
    COLOR_TABLE_SELECTED_BG: int = ...
    COLOR_MRTER_INDICATOR: int = ...
    COLOR_MRTER_TICKS: int = ...
    COLOR_CLOCK_HOUR: int = ...
    COLOR_CLOCK_MINUTE: int = ...
    COLOR_CLOCK_SECOND: int = ...
    COLOR_CLOCK_TICKS: int = ...
    COLOR_CALENDAR_YEAR: int = ...
    COLOR_CALENDAR_WEEK: int = ...
    COLOR_CALENDAR_TODAY: int = ...

    # -- Label modes --
    LABEL_AUTO: int = ...
    LABEL_DOT: int = ...
    LABEL_SCROLL: int = ...
    LABEL_SCROLL_CIRCULAR: int = ...
    LABEL_CLIP: int = ...

    # -- Meter styles --
    METER_ANGLE: int = ...
    METER_SPEED: int = ...

    # -- Clock types --
    CLOCK_DIGITAL: int = ...
    CLOCK_ANALOG: int = ...

    # -- Calendar modes --
    CALENDAR_CHINESE: int = ...
    CALENDAR_ENGLISH: int = ...

    # -- Display orientation --
    PORTRAIT: int = ...
    PORTRAIT_180: int = ...
    CROSSWISE: int = ...
    CROSSWISE_180: int = ...

    def __init__(self, title_position: int = TITLE_BOTTOM, title_high: int = 30) -> None:
        """
        Construct an IPS200PRO display object.

        :param title_position: Title bar position (TITLE_LEFT/RIGHT/TOP/BOTTOM)
        :param title_high: Title bar height [1, 200]
        """
        ...

    # -- Global methods --

    def info(self) -> None:
        """Print object information including default font size."""
        ...

    @staticmethod
    def help() -> None:
        """Print usage help."""
        ...

    def rgb888_to_rgb565(self, *args: int) -> int:
        """
        Convert RGB888 color to RGB565 format.

        Can be called as:
          rgb888_to_rgb565(rgb888_24bit)           - single 24-bit value
          rgb888_to_rgb565(red_8bit, green_8bit, blue_8bit) - separate 8-bit channels

        :return: 16-bit RGB565 color value
        """
        ...

    def set_font(self, widgets_id: int, font: int) -> None:
        """
        Set widget font size. When widgets_id=0, sets default font for new widgets.

        :param widgets_id: Widget index (0 for default)
        :param font: FONT_SIZE_xx constant
        """
        ...

    def set_color(self, widgets_id: int, type: int, color: int) -> None:
        """
        Set widget color.

        :param widgets_id: Widget index
        :param type: COLOR_xxx constant (widget-specific subset)
        :param color: RGB565 color value
        """
        ...

    def set_position(self, widgets_id: int, x: int, y: int) -> None:
        """
        Set widget position.

        :param widgets_id: Widget index
        :param x: X coordinate
        :param y: Y coordinate
        """
        ...

    def set_hidden(self, widgets_id: int, enable: bool) -> None:
        """
        Show or hide a widget.

        :param widgets_id: Widget index
        :param enable: True to hide, False to show
        """
        ...

    def set_parent(self, widgets_id1: int, widgets_id2: int) -> None:
        """
        Set parent-child relationship between widgets.
        Child position is relative to parent. Child content outside parent is clipped.
        Moving the parent moves all children.

        :param widgets_id1: Child widget index
        :param widgets_id2: Parent widget index
        """
        ...

    def set_backlight(self, backlight: int) -> None:
        """
        Set display backlight brightness.

        :param backlight: Brightness value [1, 255]
        """
        ...

    def set_dir(self, dir: int) -> None:
        """
        Set display orientation.

        :param dir: PORTRAIT, PORTRAIT_180, CROSSWISE, or CROSSWISE_180
        """
        ...

    def system_time(self, hour: int, minute: int, second: int) -> None:
        """
        Set system time for clock widget.

        :param hour: Hour [0, 23] (24-hour format)
        :param minute: Minute [0, 59]
        :param second: Second [0, 59]
        """
        ...

    def system_date(self, year: int, month: int, day: int) -> None:
        """
        Set system date for calendar widget.

        :param year: Year [1970, 2099]
        :param month: Month [1, 12]
        :param day: Day [1, max days in month]
        """
        ...

    # -- Page widget --

    def help_page(self) -> None:
        """Print page widget help."""
        ...

    def page_create(self, page_name: str) -> int:
        """
        Create a new page and switch to it immediately.

        :param page_name: Page title (supports UTF-8 Chinese/English)
        :return: Page widget index
        """
        ...

    def page_name(self, page_id: int, page_name: str) -> None:
        """
        Rename a page.

        :param page_id: Page index
        :param page_name: New page title
        """
        ...

    def page_switch(self, page_id: int, anim_enable: int = PAGE_ANIM_OFF) -> None:
        """
        Switch to a page.

        :param page_id: Page index
        :param anim_enable: PAGE_ANIM_OFF (default) or PAGE_ANIM_ON (takes ~1s)
        """
        ...

    # -- Label widget --

    def help_label(self) -> None:
        """Print label widget help."""
        ...

    def label_create(self, x: int, y: int, width: int, height: int, str: str = "", mode: int = LABEL_AUTO) -> int:
        """
        Create a label widget.

        :param x: X coordinate
        :param y: Y coordinate
        :param width: Label width
        :param height: Label height
        :param str: Initial text (optional, UTF-8)
        :param mode: Display mode (LABEL_AUTO, DOT, SCROLL, SCROLL_CIRCULAR, CLIP)
        :return: Label widget index
        """
        ...

    def label_string(self, label_id: int, str: str) -> None:
        """
        Set label text.

        :param label_id: Label index
        :param str: New text (UTF-8)
        """
        ...

    def label_mode(self, label_id: int, mode: int) -> None:
        """
        Set label display mode.

        :param label_id: Label index
        :param mode: LABEL_AUTO, DOT, SCROLL, SCROLL_CIRCULAR, or CLIP
        """
        ...

    # -- Table widget --

    def help_table(self) -> None:
        """Print table widget help."""
        ...

    def table_create(self, x: int, y: int, row: int, col: int) -> int:
        """
        Create a table widget.

        :param x: X coordinate
        :param y: Y coordinate
        :param row: Number of rows
        :param col: Number of columns
        :return: Table widget index
        """
        ...

    def table_string(self, table_id: int, row: int, col: int, str: str) -> None:
        """
        Set cell text.

        :param table_id: Table index
        :param row: Row number
        :param col: Column number
        :param str: Cell text (UTF-8)
        """
        ...

    def table_col_width(self, table_id: int, col: int, width: int) -> None:
        """
        Set column width.

        :param table_id: Table index
        :param col: Column number
        :param width: Column width in pixels
        """
        ...

    def table_select(self, table_id: int, row: int, col: int) -> None:
        """
        Select a cell, row, or column. Selection is shared across all tables.

        :param table_id: Table index
        :param row: Row number (0 = select entire column, >max = deselect)
        :param col: Column number (0 = select entire row, >max = deselect)
        """
        ...

    # -- Meter widget --

    def help_meter(self) -> None:
        """Print meter widget help."""
        ...

    def meter_create(self, x: int, y: int, diameter: int, style: int) -> int:
        """
        Create a meter widget.

        :param x: X coordinate
        :param y: Y coordinate
        :param diameter: Meter diameter (min 1)
        :param style: METER_ANGLE (0-360 degrees) or METER_SPEED (0-100 scale)
        :return: Meter widget index
        """
        ...

    def meter_value(self, meter_id: int, value: int) -> None:
        """
        Set meter value.

        :param meter_id: Meter index
        :param value: METER_ANGLE [0, 360] or METER_SPEED [0, 100]
        """
        ...

    # -- Clock widget --

    def help_clock(self) -> None:
        """Print clock widget help."""
        ...

    def clock_create(self, x: int, y: int, size: int, type: int) -> int:
        """
        Create a clock widget.

        :param x: X coordinate
        :param y: Y coordinate
        :param size: For analog: diameter [80, 240]; for digital: FONT_SIZE_xx
        :param type: CLOCK_DIGITAL or CLOCK_ANALOG
        :return: Clock widget index
        """
        ...

    # -- Progress bar widget --

    def help_progress_bar(self) -> None:
        """Print progress bar widget help."""
        ...

    def progress_bar_create(self, x: int, y: int, width: int, height: int) -> int:
        """
        Create a progress bar widget.

        :param x: X coordinate
        :param y: Y coordinate
        :param width: Bar width
        :param height: Bar height
        :return: Progress bar widget index
        """
        ...

    def progress_bar_value(self, progress_bar_id: int, start_value: int, end_value: int) -> None:
        """
        Set progress bar range.

        :param progress_bar_id: Progress bar index
        :param start_value: Start value (positive int)
        :param end_value: End value (positive int)
        """
        ...

    # -- Calendar widget --

    def help_calendar(self) -> None:
        """Print calendar widget help."""
        ...

    def calendar_create(self, x: int, y: int, width: int, height: int) -> int:
        """
        Create a calendar widget.

        :param x: X coordinate
        :param y: Y coordinate
        :param width: Calendar width
        :param height: Calendar height
        :return: Calendar widget index
        """
        ...

    def calendar_locate(self, year: int, month: int, mode: int) -> None:
        """
        Navigate calendar to a specific year and month.

        :param year: Year [1970, 2099]
        :param month: Month [1, 12]
        :param mode: CALENDAR_CHINESE or CALENDAR_ENGLISH
        """
        ...

    # -- Waveform widget --

    def help_waveform(self) -> None:
        """Print waveform widget help."""
        ...

    def waveform_create(self, x: int, y: int, width: int, height: int) -> int:
        """
        Create a waveform widget.

        :param x: X coordinate
        :param y: Y coordinate
        :param width: Waveform width
        :param height: Waveform height
        :return: Waveform widget index
        """
        ...

    def waveform_value(self, waveform_id: int, line_id: int, data, color: int = 0xF800) -> None:
        """
        Add data to a waveform line.
        Can accept a list/array of ints or a single int.

        :param waveform_id: Waveform index
        :param line_id: Line index [1, 5]
        :param data: List/array of ints [0, 100] or single int value
        :param color: Line color in RGB565 (default: red 0xF800)
        """
        ...

    def waveform_line(self, waveform_id: int, line_id: int, enable: bool = True) -> None:
        """
        Show or hide a waveform line.

        :param waveform_id: Waveform index
        :param line_id: Line index [1, 5]
        :param enable: True to show, False to hide
        """
        ...

    def waveform_mode(self, waveform_id: int, connect: bool = True) -> None:
        """
        Set waveform display mode.

        :param waveform_id: Waveform index
        :param connect: True = line (connect points), False = scatter (dot mode)
        """
        ...

    def waveform_clear(self, waveform_id: int) -> None:
        """
        Clear all waveform data.

        :param waveform_id: Waveform index
        """
        ...

    # -- Container widget --

    def help_container(self) -> None:
        """Print container widget help."""
        ...

    def container_create(self, x: int, y: int, width: int, height: int) -> int:
        """
        Create a container widget. Container has no visible content of its own
        but provides a clipping region and layout anchor for child widgets.

        :param x: X coordinate
        :param y: Y coordinate
        :param width: Container width
        :param height: Container height
        :return: Container widget index
        """
        ...

    def container_radius(self, container_id: int, border_width: int, radius: int) -> None:
        """
        Set container border style.

        :param container_id: Container index
        :param border_width: Border line width
        :param radius: Corner radius (0 for square)
        """
        ...
