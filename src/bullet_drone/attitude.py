"""IMU-only quaternion complementary filter, independent of physics and IO."""
from math import cos, hypot, isfinite, sin

from bullet_drone.config import AttitudeEstimatorSettings, _vector
from bullet_drone.contracts import LifecycleError
from bullet_drone.geometry import multiply, normalize, world_to_body
from bullet_drone.models import AttitudeEstimate, ImuReading, Quaternion, Vector3


ZERO_BIAS_RAD_S = (0.0, 0.0, 0.0)


class AttitudeEstimator:
    """Mahony proportional/integral tilt correction; yaw has no absolute reference."""

    def __init__(self, gravity_m_s2: Vector3,
                 settings: AttitudeEstimatorSettings = AttitudeEstimatorSettings()):
        gravity = _vector(gravity_m_s2, 'attitude.gravity_m_s2')
        self._gravity_norm = hypot(*gravity)
        if not isfinite(self._gravity_norm) or self._gravity_norm <= 0:
            raise ValueError('Attitude estimation requires finite nonzero gravity')
        if not isinstance(settings, AttitudeEstimatorSettings):
            raise ValueError('settings must be AttitudeEstimatorSettings')
        self._up = tuple(-v / self._gravity_norm for v in gravity)
        self._settings = settings
        self._previous = None
        self._orientation = None
        self._bias = ZERO_BIAS_RAD_S

    def _validate(self, reading: ImuReading) -> tuple[Vector3, Vector3]:
        if not isinstance(reading, ImuReading):
            raise ValueError('Attitude estimation requires an IMU reading')
        if (not isinstance(reading.episode_id, str) or not reading.episode_id.strip()
                or type(reading.step_index) is not int or reading.step_index < 0
                or type(reading.simulation_time_s) not in (int, float)
                or not isfinite(reading.simulation_time_s) or reading.simulation_time_s < 0):
            raise ValueError('Invalid IMU measurement identity or time')
        return (_vector(reading.specific_force_m_s2, 'imu.specific_force_m_s2'),
                _vector(reading.angular_velocity_rad_s, 'imu.angular_velocity_rad_s'))

    def reset(self, reading: ImuReading, orientation_xyzw: Quaternion) -> AttitudeEstimate:
        self._previous = None
        self._validate(reading)
        self._orientation = normalize(orientation_xyzw)
        self._bias = ZERO_BIAS_RAD_S
        self._previous = reading
        return self._result(reading)

    def update(self, reading: ImuReading) -> AttitudeEstimate:
        if self._previous is None:
            raise LifecycleError('Attitude estimator must be reset before updating')
        force, gyro = self._validate(reading)
        previous = self._previous
        if reading.episode_id != previous.episode_id:
            raise LifecycleError('IMU episode changed; reset the attitude estimator')
        dt = reading.simulation_time_s - previous.simulation_time_s
        if reading.step_index <= previous.step_index or dt <= 0:
            raise ValueError('IMU step and measurement time must advance')
        settings = self._settings
        error = ZERO_BIAS_RAD_S
        bias = self._bias
        magnitude = hypot(*force)
        if abs(magnitude - self._gravity_norm) <= settings.acceleration_rejection_fraction * self._gravity_norm:
            measured = tuple(v / magnitude for v in force)
            predicted = world_to_body(self._up, self._orientation)
            x, y, z = measured
            u, v, w = predicted
            error = (y*w - z*v, z*u - x*w, x*v - y*u)
            bias = tuple(b - settings.ki_s_inv2 * e * dt for b, e in zip(bias, error))
        rates = tuple(g - b + settings.kp_s_inv * e for g, b, e in zip(gyro, bias, error))
        speed = hypot(*rates)
        angle = speed * dt
        if not isfinite(angle) or any(not isfinite(b) for b in bias):
            raise ValueError('Nonfinite attitude estimator state')
        if speed > 0:
            half_angle = angle / 2
            delta = (*(v / speed * sin(half_angle) for v in rates), cos(half_angle))
            orientation = normalize(multiply(self._orientation, delta))
        else:
            orientation = self._orientation
        self._orientation, self._bias, self._previous = orientation, bias, reading
        return self._result(reading)

    def _result(self, reading: ImuReading) -> AttitudeEstimate:
        return AttitudeEstimate(reading.episode_id, reading.step_index,
                                reading.simulation_time_s, self._orientation)
