"""Aligned center-of-mass IMU, independent of physics and rendering libraries."""

from random import Random

from bullet_drone.config import ImuSettings, _vector
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

    def __init__(self, gravity_m_s2: Vector3, settings: ImuSettings = ImuSettings()) -> None:
        self._gravity = _vector(gravity_m_s2, "imu.gravity_m_s2")
        self._settings = settings
        self._rng = Random(settings.seed)
        self._previous: DroneState | None = None

    def reset(self, state: DroneState) -> ImuReading:
        self._previous = None
        self._rng.seed(self._settings.seed)
        validate_state(state)
        reading = self._measure(state, ZERO_ACCELERATION_M_S2)
        self._previous = state
        return reading

    def sample(self, state: DroneState) -> ImuReading:
        if self._previous is None:
            raise LifecycleError("IMU must be reset before sampling")
        validate_state(state, self._previous)
        dt_s = state.simulation_time_s - self._previous.simulation_time_s
        acceleration = tuple((current - previous) / dt_s for current, previous in
                             zip(state.linear_velocity_m_s, self._previous.linear_velocity_m_s))
        reading = self._measure(state, acceleration)
        self._previous = state
        return reading

    def _measure(self, state: DroneState, acceleration: Vector3) -> ImuReading:
        specific_force = world_to_body(tuple(a - g for a, g in zip(acceleration, self._gravity)),
                                       state.orientation_xyzw)
        rates = world_to_body(state.angular_velocity_rad_s, state.orientation_xyzw)
        settings = self._settings
        measured_force = tuple(noisy(value, bias, stddev, self._rng) for value, bias, stddev in
                               zip(specific_force, settings.accelerometer_bias_m_s2,
                                   settings.accelerometer_stddev_m_s2))
        measured_rates = tuple(noisy(value, bias, stddev, self._rng) for value, bias, stddev in
                               zip(rates, settings.gyro_bias_rad_s, settings.gyro_stddev_rad_s))
        return ImuReading(state.episode_id, state.step_index, state.simulation_time_s,
                          measured_force, measured_rates)
