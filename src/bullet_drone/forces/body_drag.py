"""Body-frame quadratic drag in N; air-relative velocity in m/s, CdA in m²."""

from bullet_drone.forces._math import norm
from bullet_drone.models import Vector3


DYNAMIC_PRESSURE_FACTOR = 0.5


def calculate(air_velocity_m_s: Vector3, density_kg_m3: float, cd_area_m2: Vector3) -> Vector3:
    speed = norm(air_velocity_m_s)
    return tuple(-DYNAMIC_PRESSURE_FACTOR * density_kg_m3 * area * speed * velocity
                 for area, velocity in zip(cd_area_m2, air_velocity_m_s))
