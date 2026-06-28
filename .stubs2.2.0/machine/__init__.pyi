"""
machine module for RT1021-MicroPython v2.2.0
Hardware-related functions and classes.
"""

BOARD_TYPE : str
BOARD_VERSION : str

from typing import Any, Callable, List, Optional, Union

# Module-level functions

def execfile(filename: str) -> None:
    """Execute a Python script file."""
    ...

def reset() -> None:
    """Hard reset the device."""
    ...

def soft_reset() -> None:
    """Soft reset the device (preserves some state)."""
    ...

def freq() -> int:
    """Get CPU frequency in Hz."""
    ...

def unique_id() -> bytes:
    """Get the device unique ID as bytes."""
    ...

def idle() -> None:
    """Put CPU into idle state (wakes on interrupt)."""
    ...

def disable_irq() -> Any:
    """Disable interrupts and return prior state for enable_irq."""
    ...

def enable_irq(state: Any) -> None:
    """Restore interrupt state from disable_irq."""
    ...

# Classes

class Pin:
    """GPIO pin control."""

    # Mode constants
    IN: int = ...
    OUT: int = ...
    OPEN_DRAIN: int = ...

    # Pull constants
    PULL_UP: int = ...
    PULL_UP_47K: int = ...
    PULL_UP_22K: int = ...
    PULL_DOWN: int = ...
    PULL_HOLD: int = ...

    # Drive strength constants
    DRIVE_0: int = ...
    DRIVE_1: int = ...
    DRIVE_2: int = ...
    DRIVE_3: int = ...
    DRIVE_4: int = ...
    DRIVE_5: int = ...
    DRIVE_6: int = ...
    DRIVE_OFF: int = ...

    # IRQ trigger constants
    IRQ_RISING: int = ...
    IRQ_FALLING: int = ...

    def __init__(self, pin: Optional[str], mode: int, pull: int = PULL_UP_47K, value: Union[int, bool] = 1, drive: int = DRIVE_OFF) -> None:
        """
        Construct a Pin object.

        :param pin: Pin name string (e.g. 'C4', 'B27')
        :param mode: Pin mode (Pin.IN, Pin.OUT, Pin.OPEN_DRAIN)
        :param pull: Pull configuration
        :param value: Initial output value (0/1 or False/True)
        :param drive: Drive strength
        """
        ...

    def init(self, mode: int, pull: int = PULL_UP_47K, value: Union[int, bool] = 1, drive: int = DRIVE_OFF) -> None:
        """Re-initialize the pin."""
        ...

    def value(self, x: Optional[Union[int, bool]] = None) -> Union[int, None]:
        """
        Set or get the pin value.

        :param x: Value to set (0/1 or False/True). If None, reads the pin.
        :return: Pin value when reading, None when setting.
        """
        ...

    def on(self) -> None:
        """Set pin high."""
        ...

    def off(self) -> None:
        """Set pin low."""
        ...

    def high(self) -> None:
        """Set pin high."""
        ...

    def low(self) -> None:
        """Set pin low."""
        ...

    def toggle(self) -> None:
        """Toggle pin state."""
        ...

    def irq(self, handler: Callable[['Pin'], None], trigger: int, hard: bool = False) -> None:
        """
        Configure an interrupt on this pin.

        :param handler: Callback function receiving the Pin object
        :param trigger: Pin.IRQ_RISING or Pin.IRQ_FALLING
        :param hard: True for hard interrupt context
        """
        ...


class ADC:
    """Analog-to-digital converter."""

    def __init__(self, pin: Optional[str]) -> None:
        """
        Construct an ADC object on the given pin.

        :param pin: Pin name string (e.g. 'B27', 'B12')
        """
        ...

    def read_u16(self) -> int:
        """
        Read ADC value.

        :return: 12-bit ADC value scaled to [0, 65535]
        """
        ...


class PWM:
    """Pulse-width modulation output."""

    def __init__(self, pin: Optional[str], freq: int, duty_u16: int = 0) -> None:
        """
        Construct a PWM object.

        :param pin: Pin name string
        :param freq: PWM frequency in Hz
        :param duty_u16: Initial duty cycle [1, 65535]
        """
        ...

    def init(self, freq: int, duty_u16: int = 0) -> None:
        """Re-initialize PWM."""
        ...

    def deinit(self) -> None:
        """Disable PWM output."""
        ...

    def freq(self, freq: Optional[int] = None) -> int:
        """
        Get or set PWM frequency.

        :param freq: New frequency in Hz (None to query)
        :return: Current frequency in Hz
        """
        ...

    def duty_u16(self, duty: Optional[int] = None) -> int:
        """
        Get or set PWM duty cycle.

        :param duty: New duty cycle [1, 65535] (None to query)
        :return: Current duty cycle
        """
        ...


class UART:
    """Serial communication (LPUART)."""

    def __init__(self, id: int, baudrate: int = 9600, bits: int = 8, parity: Optional[int] = None, stop: int = 1, timeout: int = 1000) -> None:
        """
        Construct a UART object.

        :param id: UART peripheral ID (0, 1, 2, 3, 5, 7)
        :param baudrate: Baud rate
        :param bits: Data bits (8)
        :param parity: None, 0 (even), or 1 (odd)
        :param stop: Stop bits (1)
        :param timeout: Read timeout in ms
        """
        ...

    def init(self, baudrate: int = 9600, bits: int = 8, parity: Optional[int] = None, stop: int = 1, timeout: int = 1000) -> None:
        """Re-initialize UART."""
        ...

    def read(self, nbytes: int) -> bytes:
        """Read up to nbytes from UART."""
        ...

    def readinto(self, buf: bytearray) -> int:
        """Read into buffer, return bytes read."""
        ...

    def readline(self) -> str:
        """Read a line from UART."""
        ...

    def write(self, buf: Union[str, bytes, bytearray]) -> int:
        """Write buffer to UART."""
        ...

    def any(self) -> int:
        """Return number of bytes available to read."""
        ...


class SPI:
    """SPI bus (LPSPI)."""

    def __init__(self, id: int, baudrate: int = 1000000, polarity: int = 0, phase: int = 0, bits: int = 8, firstbit: int = 1, sck: Optional[Pin] = None, mosi: Optional[Pin] = None, miso: Optional[Pin] = None) -> None:
        """
        Construct an SPI object.

        :param id: SPI peripheral ID [1, 3]
        :param baudrate: Clock speed in Hz
        :param polarity: Clock polarity (0 or 1)
        :param phase: Clock phase (0 or 1)
        :param bits: Bits per transfer
        :param firstbit: 1 = MSB first
        :param sck: SCK pin (optional, auto-assigned)
        :param mosi: MOSI pin (optional, auto-assigned)
        :param miso: MISO pin (optional, auto-assigned)
        """
        ...

    def init(self, baudrate: int = 1000000, polarity: int = 0, phase: int = 0, bits: int = 8, firstbit: int = 1) -> None:
        """Re-initialize SPI."""
        ...

    def read(self, nbytes: int, write: int = 0x00) -> bytes:
        """Read nbytes while writing the given value."""
        ...

    def readinto(self, buf: bytearray, write: int = 0x00) -> None:
        """Read into buffer while writing the given value."""
        ...

    def write(self, buf: Union[bytes, bytearray]) -> None:
        """Write buffer to SPI."""
        ...

    def write_readinto(self, write_buf: Union[bytes, bytearray], read_buf: bytearray) -> None:
        """Simultaneous write and read."""
        ...


class I2C:
    """Hardware I2C bus (LPI2C)."""

    def __init__(self, id: int, scl: Optional[str] = None, sda: Optional[str] = None, freq: int = 400000) -> None:
        """
        Construct an I2C object.

        :param id: I2C peripheral ID (0, 1, 3)
        :param scl: SCL pin name (optional, auto-assigned)
        :param sda: SDA pin name (optional, auto-assigned)
        :param freq: Bus frequency in Hz
        """
        ...

    def init(self, scl: Optional[str] = None, sda: Optional[str] = None, freq: int = 400000) -> None:
        """Re-initialize I2C."""
        ...

    def scan(self) -> List[int]:
        """Scan bus for devices. Returns list of 7-bit addresses."""
        ...

    def readfrom(self, addr: int, nbytes: int, stop: bool = True) -> bytes:
        """Read nbytes from device at addr."""
        ...

    def readfrom_into(self, addr: int, buf: bytearray, stop: bool = True) -> None:
        """Read into buffer from device at addr."""
        ...

    def writeto(self, addr: int, buf: Union[bytes, bytearray], stop: bool = True) -> None:
        """Write buffer to device at addr."""
        ...

    def writevto(self, addr: int, vector: List[Union[bytes, bytearray]], stop: bool = True) -> None:
        """Write vector of buffers to device at addr."""
        ...


class SoftI2C(I2C):
    """Software I2C bus."""

    def __init__(self, scl: Optional[str], sda: Optional[str], freq: int = 400000, timeout: int = 50000) -> None:
        """
        Construct a software I2C object.

        :param scl: SCL pin name
        :param sda: SDA pin name
        :param freq: Bus frequency in Hz
        :param timeout: Timeout in us
        """
        ...
