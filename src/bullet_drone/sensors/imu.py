"""Aligned center-of-mass IMU, independent of physics and rendering libraries."""

from random import Random
from math import pi, sin, sqrt

from bullet_drone.config import ConfigurationError, ImuSettings, _vector
from bullet_drone.contracts import LifecycleError
from bullet_drone.geometry import world_to_body
from bullet_drone.models import DroneState, ImuReading, Vector3
from bullet_drone.sensors._common import noisy, validate_state


ZERO_ACCELERATION_M_S2 = (0.0, 0.0, 0.0)


class ImuSensor:
    """Specific force = R^T(a_world - gravity); gyro reports body rates.

    Acceleration is the velocity difference over the completed state interval,
    expressed using the current orientation. Reset assumes zero world acceleration.
    """

    def __init__(self, gravity_m_s2: Vector3, settings: ImuSettings = ImuSettings(), *,
                 sample_period_s: float | None = None) -> None:
        self._gravity = _vector(gravity_m_s2, "imu.gravity_m_s2")
        if not isinstance(settings, ImuSettings):
            raise ConfigurationError('settings must be ImuSettings')
        if sample_period_s is None and (settings.noise_model == 'density' or settings.filter_enabled):
            raise ConfigurationError('IMU density noise or filtering requires sample_period_s')
        self._sample_period_s = (settings.validate_sample_period(sample_period_s)
                                 if sample_period_s is not None else None)
        self._settings = settings
        self._rng = Random(settings.seed)
        self._previous: DroneState | None = None
        self._force_bias = self._rate_bias = ZERO_ACCELERATION_M_S2
        self._filtered_force = self._filtered_rates = None

    def reset(self, state: DroneState) -> ImuReading:
        self._previous = None
        self._force_bias = self._rate_bias = ZERO_ACCELERATION_M_S2
        self._filtered_force = self._filtered_rates = None
        self._rng.seed(self._settings.seed)
        validate_state(state)
        reading = self._measure(state, ZERO_ACCELERATION_M_S2, self._sample_period_s)
        self._previous = state
        return reading

    def sample(self, state: DroneState) -> ImuReading:
        if self._previous is None:
            raise LifecycleError("IMU must be reset before sampling")
        validate_state(state, self._previous)
        dt_s = state.simulation_time_s - self._previous.simulation_time_s
        self._settings.validate_sample_period(dt_s)
        acceleration = tuple((current - previous) / dt_s for current, previous in
                             zip(state.linear_velocity_m_s, self._previous.linear_velocity_m_s))
        reading = self._measure(state, acceleration, dt_s)
        self._previous = state
        return reading

    def _measure(self, state: DroneState, acceleration: Vector3, dt_s: float | None) -> ImuReading:
        specific_force = world_to_body(tuple(a - g for a, g in zip(acceleration, self._gravity)),
                                       state.orientation_xyzw)
        rates = world_to_body(state.angular_velocity_rad_s, state.orientation_xyzw)
        settings = self._settings
        force_bias, rate_bias = self._force_bias, self._rate_bias
        if self._previous is not None:
            root_dt = sqrt(dt_s)
            force_bias = tuple(noisy(b, 0., walk * root_dt, self._rng) if walk else b
                               for b, walk in zip(force_bias, settings.accelerometer_bias_random_walk_m_s2_sqrt_s))
            rate_bias = tuple(noisy(b, 0., walk * root_dt, self._rng) if walk else b
                              for b, walk in zip(rate_bias, settings.gyro_bias_random_walk_rad_s_sqrt_s))
        force_stddev, rate_stddev = settings.accelerometer_stddev_m_s2, settings.gyro_stddev_rad_s
        if settings.noise_model == 'density':
            scale = 1 / sqrt(2 * dt_s)
            force_stddev = tuple(density * scale for density in settings.accelerometer_noise_density_m_s2_sqrt_hz)
            rate_stddev = tuple(density * scale for density in settings.gyro_noise_density_rad_s_sqrt_hz)
        measured_force = tuple(noisy(value, fixed + residual, stddev, self._rng)
                               for value, fixed, residual, stddev in
                               zip(specific_force, settings.accelerometer_bias_m_s2, force_bias, force_stddev))
        measured_rates = tuple(noisy(value, fixed + residual, stddev, self._rng)
                               for value, fixed, residual, stddev in
                               zip(rates, settings.gyro_bias_rad_s, rate_bias, rate_stddev))
        if settings.filter_enabled:
            measured_force = self._low_pass(measured_force, self._filtered_force, settings.accelerometer_cutoff_hz, dt_s)
            measured_rates = self._low_pass(measured_rates, self._filtered_rates, settings.gyro_cutoff_hz, dt_s)
        measured_force = _vector(measured_force, 'imu.specific_force_m_s2')
        measured_rates = _vector(measured_rates, 'imu.angular_velocity_rad_s')
        self._force_bias, self._rate_bias = force_bias, rate_bias
        self._filtered_force, self._filtered_rates = measured_force, measured_rates
        return ImuReading(state.episode_id, state.step_index, state.simulation_time_s,
                          measured_force, measured_rates)

    @staticmethod
    def _low_pass(values, previous, cutoff_hz, dt_s):
        if previous is None:
            return values
        s = sin(pi * cutoff_hz * dt_s)
        alpha = 2 * s / (sqrt(1 + s * s) + s)
        return tuple((1 - alpha) * old + alpha * value for old, value in zip(previous, values))
