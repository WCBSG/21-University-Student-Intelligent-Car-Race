"""
display module for RT1021-MicroPython v2.2.0
LCD display drivers for IPS200 and IPS114 screens.
"""

from typing import List, Optional
from array import array
from machine import Pin


class LCD_Drv:
    """LCD driver interface for low-level SPI communication."""

    LCD200_TYPE: int = ...
    LCD114_TYPE: int = ...

    def __init__(self, SPI_INDEX: int, BAUDRATE: int, DC_PIN: Pin, RST_PIN: Pin, LCD_TYPE: int = LCD200_TYPE) -> None:
        """
        Construct an LCD driver.

        :param SPI_INDEX: SPI peripheral ID (2 for LPSPI3 on most boards)
        :param BAUDRATE: SPI clock speed (e.g. 60000000 for 60MHz)
        :param DC_PIN: Data/Command select pin
        :param RST_PIN: Reset pin
        :param LCD_TYPE: LCD200_TYPE or LCD114_TYPE
        """
        ...


class LCD:
    """High-level LCD display operations."""

    def __init__(self, drv: LCD_Drv) -> None:
        """
        Construct an LCD object.

        :param drv: LCD_Drv instance
        """
        ...

    def color(self, pcolor: int, bgcolor: int) -> None:
        """
        Set global foreground and background colors.

        :param pcolor: Foreground color (RGB565)
        :param bgcolor: Background color (RGB565)
        """
        ...

    def mode(self, dir: int) -> None:
        """
        Set display orientation.

        :param dir: 0=portrait, 1=landscape, 2=portrait 180, 3=landscape 180
        """
        ...

    def clear(self, color: Optional[int] = None) -> None:
        """
        Clear the screen.

        :param color: Fill color (RGB565), uses bgcolor if None
        """
        ...

    def str12(self, x: int, y: int, str: str, color: Optional[int] = None) -> None:
        """
        Draw text with 12-pixel font.

        :param x: X coordinate
        :param y: Y coordinate
        :param str: Text string (supports Python format specifiers)
        :param color: Text color (RGB565), uses global pcolor if None
        """
        ...

    def str16(self, x: int, y: int, str: str, color: Optional[int] = None) -> None:
        """
        Draw text with 16-pixel font.

        :param x: X coordinate
        :param y: Y coordinate
        :param str: Text string
        :param color: Text color (RGB565)
        """
        ...

    def str24(self, x: int, y: int, str: str, color: Optional[int] = None) -> None:
        """
        Draw text with 24-pixel font.

        :param x: X coordinate
        :param y: Y coordinate
        :param str: Text string
        :param color: Text color (RGB565)
        """
        ...

    def str32(self, x: int, y: int, str: str, color: Optional[int] = None) -> None:
        """
        Draw text with 32-pixel font.

        :param x: X coordinate
        :param y: Y coordinate
        :param str: Text string
        :param color: Text color (RGB565)
        """
        ...

    def line(self, x1: int, y1: int, x2: int, y2: int, color: Optional[int] = None, thick: int = 1) -> None:
        """
        Draw a line.

        :param x1: Start X
        :param y1: Start Y
        :param x2: End X
        :param y2: End Y
        :param color: Line color (RGB565)
        :param thick: Line thickness
        """
        ...

    def wave(self, x: int, y: int, width: int, height: int, data: 'array', max: int) -> None:
        """
        Display a waveform from a sequence of integers.

        :param x: X coordinate
        :param y: Y coordinate
        :param width: Waveform width in pixels
        :param height: Waveform height in pixels
        :param data: Array ('h' typecode) of int values
        :param max: Maximum value for scaling
        """
        ...
