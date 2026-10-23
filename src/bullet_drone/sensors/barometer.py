"""Constant-temperature atmospheric pressure and pressure-derived altitude."""

from math import exp, expm1, isfinite, log
from random import Random

from bullet_drone.config import BarometerSettings
from bullet_drone.contracts import LifecycleError
from bullet_drone.models import BarometerReading, DroneState
from bullet_drone.sensors._common import noisy, validate_state


ATMOSPHERIC_GRAVITY_M_S2 = 9.80665
DRY_AIR_GAS_CONSTANT_J_KG_K = 287.05
WORLD_Z_AXIS = 2


class BarometerSensor:
    """Earth atmosphere in world Z; optional pressure smoothing before inversion.

    The local isothermal atmosphere is a low-altitude approximation, independent
    of configured physics gravity. No weather, rotor wash or temperature noise.
    """

    def __init__(self, settings: BarometerSettings = BarometerSettings()) -> None:
        self._settings = settings
        self._scale_height_m = (DRY_AIR_GAS_CONSTANT_J_KG_K * settings.reference_temperature_k
                                / ATMOSPHERIC_GRAVITY_M_S2)
        if not isfinite(self._scale_height_m) or self._scale_height_m <= 0:
            raise ValueError("Barometer scale height must be positive and finite")
        self._rng = Random(settings.seed)
        self._previous: DroneState | None = None
        self._filtered_pressure_pa: float | None = None

    def reset(self, state: DroneState) -> BarometerReading:
        self._previous = None
        self._filtered_pressure_pa = None
        self._rng.seed(self._settings.seed)
        validate_state(state)
        reading = self._measure(state)
        self._previous = state
        return reading

    def sample(self, state: DroneState) -> BarometerReading:
        if self._previous is None:
            raise LifecycleError("Barometer must be reset before sampling")
        validate_state(state, self._previous)
        reading = self._measure(state)
        self._previous = state
        return reading

    def _measure(self, state: DroneState) -> BarometerReading:
        settings = self._settings
        height_m = state.position_m[WORLD_Z_AXIS] - settings.reference_altitude_m
        if not isfinite(height_m):
            raise ValueError("Barometer source height must be finite")
        try:
            pressure = settings.reference_pressure_pa * exp(-height_m / self._scale_height_m)
        except OverflowError as exc:
            raise ValueError("Barometer source height exceeds the pressure model range") from exc
        pressure = noisy(pressure, settings.pressure_bias_pa, settings.pressure_stddev_pa, self._rng)
        if not isfinite(pressure) or pressure <= 0:
            raise ValueError("Measured barometric pressure must be positive and finite")
        if settings.filter_enabled and self._filtered_pressure_pa is not None:
            dt = state.simulation_time_s - self._previous.simulation_time_s
            alpha = -expm1(-dt / settings.filter_time_constant_s)
            pressure = (1 - alpha) * self._filtered_pressure_pa + alpha * pressure
            if not isfinite(pressure) or pressure <= 0:
                raise ValueError("Filtered barometric pressure must be positive and finite")
        altitude = (settings.reference_altitude_m
                    - self._scale_height_m * (log(pressure) - log(settings.reference_pressure_pa)))
        if not isfinite(altitude):
            raise ValueError("Barometer altitude must be finite")
        if settings.filter_enabled:
            self._filtered_pressure_pa = pressure
        return BarometerReading(state.episode_id, state.step_index, state.simulation_time_s,
                                pressure, altitude)
