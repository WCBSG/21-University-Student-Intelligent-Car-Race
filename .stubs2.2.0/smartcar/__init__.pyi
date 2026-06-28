"""
smartcar module for RT1021-MicroPython v2.2.0
Timer-based scheduling (ticker), ADC group sampling, and encoder interface.
"""

from typing import Any, Callable, List, Optional
from seekfree import KEY_HANDLER, IMU660RX, IMU963RX, DL1X, TSL1401


class ticker:
    """Hardware timer interrupt (PIT) for periodic tasks."""

    def __init__(self, id: int) -> None:
        """
        Construct a ticker object.

        :param id: PIT channel ID [0, 3]
        """
        ...

    def start(self, period_ms: int) -> None:
        """
        Start the ticker.

        :param period_ms: Tick period in ms (minimum 5ms)
        """
        ...

    def stop(self) -> None:
        """Stop the ticker."""
        ...

    def callback(self, handler: Callable[['ticker'], None]) -> None:
        """
        Register a callback function called on each tick.

        :param handler: Function receiving the ticker instance
        """
        ...

    def capture_list(self, *modules: Any) -> None:
        """
        Register sensor/controller instances for automatic capture on each tick.
        Supported types: ADC_Group, encoder, KEY_HANDLER, IMU660RX, IMU963RX, DL1X, TSL1401.

        :param modules: One or more sensor objects
        """
        ...

    def ticks(self) -> int:
        """
        Get cumulative tick count.

        :return: Number of ticks since start
        """
        ...


class ADC_Group:
    """ADC group sampling for multi-channel analog capture (e.g. electromagnetic sensors)."""

    # Period modes
    PMODE0: int = ...
    PMODE1: int = ...
    PMODE2: int = ...
    PMODE3: int = ...

    # Averaging counts
    AVG1: int = ...
    AVG4: int = ...
    AVG8: int = ...
    AVG16: int = ...
    AVG32: int = ...

    def __init__(self, id: int) -> None:
        """
        Construct an ADC_Group object.

        :param id: ADC group ID [1, 2]. ADC_Group1 uses ADC1, ADC_Group2 uses ADC2.
        """
        ...

    def init(self, id: int, period: int = PMODE3, average: int = AVG16) -> None:
        """
        Initialize the ADC group.

        :param id: ADC group ID [1, 2]
        :param period: Sampling period mode (PMODE0-PMODE3)
        :param average: Averaging count (AVG1-AVG32)
        """
        ...

    def addch(self, pin: Optional[str]) -> None:
        """
        Add an ADC channel.

        :param pin: Pin name string (e.g. 'B12')
        """
        ...

    def capture(self) -> None:
        """Add a capture request (called by ticker or manually)."""
        ...

    def get(self) -> List[int]:
        """
        Get ADC data buffer (linked list - auto-updates on capture).

        :return: List of ADC values
        """
        ...

    def read(self) -> List[int]:
        """Immediate capture and read ADC values."""
        ...


class encoder:
    """Quadrature encoder interface."""

    def __init__(self, PhaseA: Optional[str], PhaseB: Optional[str], invert: bool = False, capture_div: int = 1) -> None:
        """
        Construct an encoder object.

        :param PhaseA: Phase A pin name (PLUS)
        :param PhaseB: Phase B pin name (DIR)
        :param invert: Invert counting direction
        :param capture_div: Capture divider (n ticker ticks per capture)
        """
        ...

    def capture(self) -> None:
        """Add a capture request (called by ticker or manually)."""
        ...

    def get(self) -> int:
        """
        Get encoder count (linked - auto-updates on capture).

        :return: Current encoder count
        """
        ...

    def read(self) -> int:
        """Immediate capture and read encoder count."""
        ...
