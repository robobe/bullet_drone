"""Ideal ESC/motor response; target and actual angular speeds are rad/s."""

from bullet_drone.commands import InvalidCommandError, _finite, _values
from bullet_drone.config import _number
from bullet_drone.models import RotorSpeeds


class IdealMotorDynamics:
    def __init__(self, max_speed_rad_s: float) -> None:
        self.max_speed_rad_s = _number(max_speed_rad_s, "max_speed_rad_s", positive=True)

    def reset(self) -> RotorSpeeds:
        return (0.0, 0.0, 0.0, 0.0)

    def update(self, targets_rad_s: RotorSpeeds, dt_s: float) -> RotorSpeeds:
        targets = _values(targets_rad_s, 4, "targets_rad_s")
        dt_s = _finite(dt_s, "dt_s")
        if dt_s <= 0:
            raise InvalidCommandError("dt_s must be positive")
        if any(v < 0 or v > self.max_speed_rad_s for v in targets):
            raise InvalidCommandError("Motor target exceeds speed limits")
        return targets
