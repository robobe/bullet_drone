"""Body torque in N*m opposing body angular velocity; coefficients N*m/(rad/s)."""

from bullet_drone.models import Vector3


def calculate(body_rates_rad_s: Vector3, coefficients_nm_per_rad_s: Vector3) -> Vector3:
    return tuple(-coefficient * rate for coefficient, rate in zip(coefficients_nm_per_rad_s, body_rates_rad_s))
