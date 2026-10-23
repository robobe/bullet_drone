"""Bounded teaching-model thrust reduction from horizontal rotor advance ratio."""

from math import hypot

from bullet_drone.models import Vector3


def multiplier(air_velocity_m_s: Vector3, speed_rad_s: float, radius_m: float,
               coefficient: float, minimum_multiplier: float) -> float:
    if speed_rad_s <= 0:
        return 1.0
    tip_speed = radius_m * speed_rad_s
    return max(minimum_multiplier, min(1.0, 1 - coefficient * hypot(*air_velocity_m_s[:2]) / tip_speed))
