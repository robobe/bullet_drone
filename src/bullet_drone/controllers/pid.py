"""Scalar PID with derivative on measurement; callers define input/output units."""

from dataclasses import dataclass

from bullet_drone.config import ConfigurationError, _number
from bullet_drone.commands import InvalidCommandError, _finite


def clamp(value: float, limit: float) -> float:
    return min(limit, max(-limit, value))


@dataclass(frozen=True, slots=True)
class PidSettings:
    kp: float
    ki: float
    kd: float
    output_limit: float
    integral_limit: float

    def __post_init__(self) -> None:
        for name in ("kp", "ki", "kd", "output_limit", "integral_limit"):
            value = _number(getattr(self, name), f"pid.{name}", positive=name == "output_limit")
            if value < 0:
                raise ConfigurationError(f"pid.{name} cannot be negative")
            object.__setattr__(self, name, value)
        if self.integral_limit > self.output_limit:
            raise ConfigurationError("PID integral limit cannot exceed output limit")


class PID:
    """Integral state is an output contribution; reset clears all history.

    kp: output/input; ki: output/(input*s); kd: output*s/input.
    Feedforward uses output units and participates in saturation protection.
    Anti-windup covers PID output saturation; downstream mixer saturation is
    reported separately and does not back-calculate integral state.
    """

    def __init__(self, settings: PidSettings) -> None:
        self.settings = settings
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self._previous_measurement: float | None = None

    def update(self, setpoint: float, measurement: float, dt_s: float, feedforward: float = 0.0) -> float:
        setpoint = _finite(setpoint, "setpoint")
        measurement = _finite(measurement, "measurement")
        dt_s = _finite(dt_s, "dt_s")
        feedforward = _finite(feedforward, "feedforward")
        if dt_s <= 0:
            raise InvalidCommandError("dt_s must be positive")
        s = self.settings
        error = _finite(setpoint - measurement, "PID error")
        derivative = 0.0 if self._previous_measurement is None else -(measurement - self._previous_measurement) / dt_s
        base = _finite(s.kp * error + s.kd * derivative + feedforward, "PID output")
        increment = _finite(s.ki * error * dt_s, "PID integral increment")
        candidate = clamp(_finite(self.integral + increment, "PID integral"), s.integral_limit)
        total = _finite(base + candidate, "PID output")
        # Reject integration that drives saturation further; allow unwinding.
        if not ((total > s.output_limit and increment > 0) or (total < -s.output_limit and increment < 0)):
            self.integral = candidate
        output = clamp(_finite(base + self.integral, "PID output"), s.output_limit)
        self._previous_measurement = measurement
        return output
