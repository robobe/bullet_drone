"""Resolved SI, PyBullet-world poses; no backend handles cross this boundary."""
from dataclasses import dataclass
from math import hypot
from bullet_drone.rendering.config import RenderSettings, RenderSources, identity, number, vector
from bullet_drone.models import DroneObservation


@dataclass(frozen=True, slots=True)
class RenderSnapshot:
    drone_id: str
    episode_id: str
    step_index: int
    simulation_time_s: float
    position_m: tuple
    orientation_xyzw: tuple
    sources: RenderSources

    def __post_init__(self):
        identity(self.drone_id, 'drone_id')
        identity(self.episode_id, 'episode_id')
        if type(self.step_index) is not int or self.step_index < 0:
            raise ValueError('step_index must be a nonnegative integer')
        number(self.simulation_time_s, 'simulation_time_s')
        if self.simulation_time_s < 0:
            raise ValueError('simulation_time_s must be nonnegative')
        object.__setattr__(self, 'position_m', vector(self.position_m, 3, 'position_m'))
        q = vector(self.orientation_xyzw, 4, 'orientation_xyzw')
        scale = max(abs(v) for v in q)
        if scale == 0:
            raise ValueError('orientation quaternion must be nonzero')
        scaled = tuple(v / scale for v in q)
        norm = hypot(*scaled)
        object.__setattr__(self, 'orientation_xyzw', tuple(v / norm for v in scaled))
        if not isinstance(self.sources, RenderSources):
            raise ValueError('sources must be RenderSources')

    @property
    def altitude_m(self):
        return self.position_m[2]


class SnapshotBuilder:
    def __init__(self, settings: RenderSettings):
        self.settings = settings

    def build(self, observation: DroneObservation) -> RenderSnapshot:
        state = observation.ground_truth
        # All supported selections are ground truth; no sensor fallback exists.
        return RenderSnapshot(self.settings.drone_id, state.episode_id, state.step_index,
                              state.simulation_time_s, state.position_m,
                              state.orientation_xyzw, self.settings.sources)
