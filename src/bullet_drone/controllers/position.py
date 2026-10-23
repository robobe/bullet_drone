"""Horizontal position PID: world XY meters → world XY velocity in m/s."""

from math import hypot

from bullet_drone.commands import InvalidCommandError, _finite, _values
from bullet_drone.controllers.pid import PID, PidSettings


XY_AXIS_COUNT = 2


class PositionController:
    """Independent axis PIDs with a vector speed limit; no attitude or RC logic."""

    def __init__(self, position_pid: PidSettings) -> None:
        self._pids = tuple(PID(position_pid) for _ in range(XY_AXIS_COUNT))
        self.last_output: tuple[float, float] | None = None

    def reset(self) -> None:
        self.last_output = None
        for pid in self._pids:
            pid.reset()

    def velocity(self, target_position_xy_m: tuple[float, float],
                 position_xy_m: tuple[float, float], dt_s: float) -> tuple[float, float]:
        target = _values(target_position_xy_m, XY_AXIS_COUNT, 'target_position_xy_m')
        position = _values(position_xy_m, XY_AXIS_COUNT, 'position_xy_m')
        dt = _finite(dt_s, 'dt_s')
        if dt <= 0:
            raise InvalidCommandError('dt_s must be positive')
        speeds = tuple(pid.update(t, p, dt) for pid, t, p in zip(self._pids, target, position))
        magnitude = hypot(*speeds)
        limit = self._pids[0].settings.output_limit
        factor = limit / magnitude if magnitude > limit else 1.0
        self.last_output = tuple(speed * factor for speed in speeds)
        return self.last_output
