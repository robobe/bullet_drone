"""Teaching-model in-plane rotor force in body N, opposite horizontal airflow."""

from bullet_drone.forces._math import ZERO_VECTOR
from bullet_drone.models import Vector3


def calculate(air_velocity_m_s: Vector3, thrust_n: float, speed_rad_s: float,
              radius_m: float, coefficient: float) -> Vector3:
    if speed_rad_s <= 0:
        return ZERO_VECTOR
    factor = -coefficient * max(0.0, thrust_n) / (radius_m * speed_rad_s)
    return (factor * air_velocity_m_s[0], factor * air_velocity_m_s[1], 0.0)
