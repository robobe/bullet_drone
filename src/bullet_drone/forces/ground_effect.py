"""Bounded thrust gain from a supporting surface; no force application or rays."""

from bullet_drone.config import GroundEffectSettings
from bullet_drone.forces._math import dot
from bullet_drone.models import SurfaceHit, Vector3


ROTOR_DIAMETER_FACTOR = 2.0


def multiplier(hit: SurfaceHit | None, rotor_axis_world: Vector3, radius_m: float,
               settings: GroundEffectSettings) -> float:
    if hit is None or hit.distance_m > settings.query_range_m:
        return 1.0
    alignment = max(0.0, dot(rotor_axis_world, hit.normal_world))
    distance = max(settings.minimum_distance_m, hit.distance_m)
    gain = settings.coefficient * (radius_m / (ROTOR_DIAMETER_FACTOR * distance)) ** 2 * alignment ** 2
    return min(settings.maximum_multiplier, 1 + gain)
