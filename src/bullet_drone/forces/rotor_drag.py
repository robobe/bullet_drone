"""Body-frame linear drag in N; coefficients use actual rad/s and airspeed m/s."""

from bullet_drone.models import RotorSpeeds, Vector3


def calculate(air_velocity_m_s: Vector3, speeds_rad_s: RotorSpeeds, coefficients: Vector3) -> Vector3:
    total_speed = sum(speeds_rad_s)
    return tuple(-coefficient * total_speed * velocity
                 for coefficient, velocity in zip(coefficients, air_velocity_m_s))
