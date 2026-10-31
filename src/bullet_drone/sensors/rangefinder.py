"""Body-down terrain rangefinder."""

from bullet_drone.config import RangefinderSettings
from bullet_drone.contracts import LifecycleError, SurfaceQueries
from bullet_drone.geometry import body_to_world
from bullet_drone.models import DroneState, RangefinderReading
from bullet_drone.sensors._common import validate_state


class RangefinderSensor:
    def __init__(self, surfaces: SurfaceQueries, settings: RangefinderSettings) -> None:
        self._surfaces = surfaces
        self._settings = settings
        self._previous: DroneState | None = None

    def reset(self, state: DroneState) -> RangefinderReading:
        self._previous = None
        validate_state(state)
        reading = self._measure(state)
        self._previous = state
        return reading

    def sample(self, state: DroneState) -> RangefinderReading:
        if self._previous is None:
            raise LifecycleError("Rangefinder must be reset before sampling")
        validate_state(state, self._previous)
        reading = self._measure(state)
        self._previous = state
        return reading

    def _measure(self, state: DroneState) -> RangefinderReading:
        origin = state.position_m
        offset = body_to_world(self._settings.mount_position_body_m, state.orientation_xyzw)
        origin = tuple(position + delta for position, delta in zip(origin, offset))
        direction = body_to_world((0.0, 0.0, -1.0), state.orientation_xyzw)
        hit = self._surfaces.ray_surface(origin, direction, self._settings.max_range_m)
        if hit is None:
            return RangefinderReading(state.episode_id, state.step_index, state.simulation_time_s, None, False)
        return RangefinderReading(state.episode_id, state.step_index, state.simulation_time_s,
                                  hit.distance_m * 100.0, True)
