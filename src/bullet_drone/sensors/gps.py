"""Ideal GPS in a fixed local east/north/up frame."""
from math import cos, degrees, hypot, isfinite, radians, sin, sqrt

from bullet_drone.config import GpsSettings, _vector
from bullet_drone.contracts import LifecycleError
from bullet_drone.models import DroneState, GpsReading
from bullet_drone.sensors._common import validate_state


WGS84_SEMI_MAJOR_AXIS_M = 6378137.0
WGS84_ECCENTRICITY_SQUARED = 6.6943799901413165e-3
MAX_LOCAL_DISTANCE_M = 1000.0


class GpsSensor:
    def __init__(self, settings: GpsSettings) -> None:
        if any(getattr(settings, name) is None for name in (
                'origin_latitude_deg', 'origin_longitude_deg', 'origin_altitude_m')):
            raise ValueError('GPS sensor requires a complete geographic origin')
        self._settings = settings
        latitude = radians(settings.origin_latitude_deg)
        curvature = 1 - WGS84_ECCENTRICITY_SQUARED * sin(latitude) ** 2
        self._north_radius_m = WGS84_SEMI_MAJOR_AXIS_M * (1 - WGS84_ECCENTRICITY_SQUARED) / curvature ** 1.5
        self._east_radius_m = WGS84_SEMI_MAJOR_AXIS_M / sqrt(curvature) * cos(latitude)
        self._previous: DroneState | None = None

    def reset(self, state: DroneState) -> GpsReading:
        self._previous = None
        return self._read(state)

    def sample(self, state: DroneState) -> GpsReading:
        if self._previous is None:
            raise LifecycleError('GPS must be reset before sampling')
        return self._read(state)

    def _read(self, state: DroneState) -> GpsReading:
        validate_state(state, self._previous)
        east, north, up = _vector(state.position_m, 'gps.position_m')
        velocity = _vector(state.linear_velocity_m_s, 'gps.velocity_enu_m_s')
        # shortcut: fixed tangent approximation within 1 km, use geodesy for larger worlds.
        if hypot(east, north) > MAX_LOCAL_DISTANCE_M:
            raise ValueError('GPS local tangent model supports positions within 1 km of origin')
        settings = self._settings
        latitude = settings.origin_latitude_deg + degrees(north / self._north_radius_m)
        longitude = (settings.origin_longitude_deg + degrees(east / self._east_radius_m) + 180) % 360 - 180
        altitude = settings.origin_altitude_m + up
        if not isfinite(altitude) or not -90 <= latitude <= 90:
            raise ValueError('GPS position is outside geographic bounds')
        reading = GpsReading(state.episode_id, state.step_index, state.simulation_time_s,
                             latitude, longitude, altitude, velocity)
        self._previous = state
        return reading
