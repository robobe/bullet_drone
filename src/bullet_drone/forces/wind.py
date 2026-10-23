"""World air velocity in m/s: constant mean plus simulation-time OU gusts."""

from math import exp, expm1, isfinite, sqrt
from random import Random

from bullet_drone.config import WindSettings
from bullet_drone.forces._math import ZERO_VECTOR, add
from bullet_drone.models import Vector3


OU_VARIANCE_EXPONENT = 2.0


class WindModel:
    def __init__(self, settings: WindSettings) -> None:
        self._settings = settings
        self._rng = Random(settings.seed)
        self.reset()

    def reset(self) -> None:
        self._rng.seed(self._settings.seed)
        self._gust = ZERO_VECTOR
        self._time_s = 0.0

    def velocity(self, simulation_time_s: float) -> Vector3:
        if not isfinite(simulation_time_s) or simulation_time_s < self._time_s:
            raise ValueError("Wind simulation time must be finite and cannot go backwards")
        dt_s = simulation_time_s - self._time_s
        self._time_s = simulation_time_s
        settings = self._settings
        if not settings.enabled:
            return ZERO_VECTOR
        if settings.behavior == "gusts" and dt_s > 0:
            ratio = dt_s / settings.correlation_time_s
            decay = exp(-ratio)
            deviation = sqrt(-expm1(-OU_VARIANCE_EXPONENT * ratio))
            self._gust = tuple(decay * value + sigma * deviation * self._rng.gauss(0.0, 1.0)
                               for value, sigma in zip(self._gust, settings.gust_stddev_m_s))
        return add(settings.velocity_world_m_s, self._gust)
