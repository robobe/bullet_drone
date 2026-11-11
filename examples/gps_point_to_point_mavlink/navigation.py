"""GPS/local tangent conversion and straight-line guidance, metres in ENU."""
from math import cos, degrees, hypot, isfinite, radians, sin, sqrt

from bullet_drone.sensors.gps import WGS84_SEMI_MAJOR_AXIS_M, WGS84_ECCENTRICITY_SQUARED

MAX_DISTANCE_M = 1000.0
LOOKAHEAD_M = 8.0
CRUISE_SPEED_M_S = 5.0
BRAKING_ACCELERATION_M_S2 = 1.0
POSITION_GAIN_S_INV = 0.8


class GeographicFrame:
    def __init__(self, latitude_deg, longitude_deg, altitude_m):
        if not all(isfinite(v) for v in (latitude_deg, longitude_deg, altitude_m)):
            raise ValueError('GPS origin must be finite')
        if abs(latitude_deg) > 85 or abs(longitude_deg) > 180:
            raise ValueError('GPS origin out of range')
        self.origin = latitude_deg, longitude_deg, altitude_m
        latitude = radians(latitude_deg)
        curvature = 1 - WGS84_ECCENTRICITY_SQUARED * sin(latitude) ** 2
        self.north_radius = WGS84_SEMI_MAJOR_AXIS_M * (1 - WGS84_ECCENTRICITY_SQUARED) / curvature ** 1.5
        self.east_radius = WGS84_SEMI_MAJOR_AXIS_M / sqrt(curvature) * cos(latitude)

    def local(self, latitude_deg, longitude_deg, altitude_m):
        if not all(isfinite(v) for v in (latitude_deg, longitude_deg, altitude_m)):
            raise ValueError('GPS position must be finite')
        if abs(latitude_deg) > 90 or abs(longitude_deg) > 180:
            raise ValueError('GPS position out of range')
        lat, lon, alt = self.origin
        east = radians((longitude_deg - lon + 180) % 360 - 180) * self.east_radius
        north = radians(latitude_deg - lat) * self.north_radius
        if hypot(east, north) > MAX_DISTANCE_M:
            raise ValueError('GPS route exceeds local 1 km model')
        return east, north, altitude_m - alt


def line_velocity(start, end, measured):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = hypot(dx, dy)
    if length == 0:
        raise ValueError('Route endpoints must differ')
    along = ((measured[0] - start[0]) * dx + (measured[1] - start[1]) * dy) / length
    target_along = min(length, max(0.0, along) + LOOKAHEAD_M)
    target = (start[0] + dx * target_along / length, start[1] + dy * target_along / length)
    error = target[0] - measured[0], target[1] - measured[1]
    distance = hypot(end[0] - measured[0], end[1] - measured[1])
    speed = min(CRUISE_SPEED_M_S, sqrt(2 * BRAKING_ACCELERATION_M_S2 * distance),
                POSITION_GAIN_S_INV * hypot(*error))
    magnitude = hypot(*error)
    return tuple(speed * v / magnitude for v in error) if magnitude else (0.0, 0.0)
