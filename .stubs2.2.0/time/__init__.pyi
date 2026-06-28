"""
time module for RT1021-MicroPython v2.2.0
Time-related functions.
"""

from typing import Optional, Tuple


def localtime(secs: Optional[int] = None) -> Tuple[int, int, int, int, int, int, int, int]:
    """
    Convert a timestamp to a time tuple.

    :param secs: Unix timestamp (uses RTC if None)
    :return: (year, month, day, hour, minute, second, weekday, yearday)
    """
    ...

def mktime(t: Tuple[int, int, int, int, int, int, int, int]) -> int:
    """
    Convert a time tuple to a Unix timestamp.

    :param t: Time tuple (year, month, day, hour, minute, second, weekday, yearday)
    :return: Unix timestamp
    """
    ...

def sleep(seconds: float) -> None:
    """
    Sleep for a number of seconds.

    :param seconds: Sleep duration (accepts float)
    """
    ...

def sleep_ms(ms: int) -> None:
    """
    Sleep for a number of milliseconds.

    :param ms: Milliseconds
    """
    ...

def sleep_us(us: int) -> None:
    """
    Sleep for a number of microseconds.

    :param us: Microseconds
    """
    ...

def ticks_ms() -> int:
    """Return millisecond counter (wraps every ~298 hours)."""
    ...

def ticks_us() -> int:
    """Return microsecond counter (wraps every ~71 minutes)."""
    ...

def ticks_cpu() -> int:
    """Return CPU cycle counter."""
    ...

def ticks_add(ticks: int, delta: int) -> int:
    """
    Add delta to a ticks value (overflow-safe).

    :param ticks: Base ticks value
    :param delta: Offset to add
    :return: Offset ticks value
    """
    ...

def ticks_diff(ticks1: int, ticks2: int) -> int:
    """
    Difference of two tick values (overflow-safe).

    :param ticks1: Later ticks value
    :param ticks2: Earlier ticks value
    :return: Signed difference in ticks
    """
    ...

def time() -> float:
    """Return current Unix timestamp in seconds (float)."""
    ...
