"""Quasi-steady downstream rotor wakes in world m/s; excludes own-rotor inflow."""

from math import exp, pi, sqrt

from bullet_drone.config import PropwashSettings
from bullet_drone.forces._math import ZERO_VECTOR, add, dot, norm, scale
from bullet_drone.models import Vector3


MOMENTUM_FLUX_FACTOR = 2.0
FAR_WAKE_SPEED_FACTOR = 2.0
RADIAL_DECAY_FACTOR = 2.0
MINIMUM_DOWNSTREAM_DISTANCE_M = 1e-9


def velocity_at(point_world_m: Vector3, rotor_positions_world_m: tuple[Vector3, ...],
                rotor_axis_world: Vector3, thrusts_n: tuple[float, ...], density_kg_m3: float,
                radius_m: float, settings: PropwashSettings, *, exclude_rotor: int | None = None) -> Vector3:
    down = scale(rotor_axis_world, -1)
    velocity = ZERO_VECTOR
    for index, (origin, thrust) in enumerate(zip(rotor_positions_world_m, thrusts_n)):
        if index == exclude_rotor or thrust <= 0:
            continue
        delta = add(point_world_m, scale(origin, -1))
        downstream = dot(delta, down)
        if downstream <= MINIMUM_DOWNSTREAM_DISTANCE_M:
            continue
        radial = add(delta, scale(down, -downstream))
        width = radius_m + settings.spread * downstream
        induced = sqrt(thrust / (MOMENTUM_FLUX_FACTOR * density_kg_m3 * pi * radius_m ** 2))
        speed = (FAR_WAKE_SPEED_FACTOR * induced * (radius_m / width) ** 2
                 * exp(-RADIAL_DECAY_FACTOR * dot(radial, radial) / width ** 2))
        velocity = add(velocity, scale(down, speed))
    speed = norm(velocity)
    return scale(velocity, min(1.0, settings.maximum_speed_m_s / speed)) if speed > 0 else ZERO_VECTOR
