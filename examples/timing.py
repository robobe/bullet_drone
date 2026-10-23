"""Wall-clock diagnostics shared by examples; callers own stepping and pacing."""

from collections.abc import Callable
import time

from loguru import logger


REPORT_INTERVAL_S = 1.0
LOG_PRECISION = 3


class RealtimeFactorLogger:
    """Report cumulative simulated seconds per wall second from a loop's origin."""

    def __init__(self, initial_simulation_time_s: float, started_wall_time_s: float, *,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._initial_simulation_time_s = initial_simulation_time_s
        self._started_wall_time_s = started_wall_time_s
        self._last_report_wall_time_s = started_wall_time_s
        self._clock = clock

    def update(self, simulation_time_s: float, *, final: bool = False) -> None:
        now = self._clock()
        if not final and now - self._last_report_wall_time_s < REPORT_INTERVAL_S:
            return
        wall_elapsed_s = now - self._started_wall_time_s
        simulation_elapsed_s = simulation_time_s - self._initial_simulation_time_s
        factor = (f"{simulation_elapsed_s / wall_elapsed_s:.{LOG_PRECISION}f}x"
                  if wall_elapsed_s > 0 else "n/a")
        logger.info("{}Timing: sim={}s wall={}s RTF={}", "Final " if final else "",
                    f"{simulation_elapsed_s:.{LOG_PRECISION}f}",
                    f"{wall_elapsed_s:.{LOG_PRECISION}f}", factor)
        self._last_report_wall_time_s = now
