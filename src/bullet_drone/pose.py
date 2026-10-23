"""Select published pose without modifying the simulated physical state."""
from dataclasses import replace

from bullet_drone.config import PoseSources, _vector
from bullet_drone.geometry import normalize
from bullet_drone.models import DroneObservation, DroneState


def resolve_pose(observation: DroneObservation, sources: PoseSources) -> DroneState:
    if not isinstance(sources, PoseSources):
        raise ValueError('sources must be PoseSources')
    state = observation.ground_truth
    if sources == PoseSources():
        return state
    readings = []
    if sources.altitude == 'barometer':
        readings.append(observation.sensors.barometer)
    if sources.attitude == 'imu':
        readings.append(observation.attitude_estimate)
    for reading in readings:
        if (reading is None or (reading.episode_id, reading.step_index, reading.simulation_time_s) !=
                (state.episode_id, state.step_index, state.simulation_time_s)):
            raise ValueError('Selected pose source is missing or does not match the physics observation')
    position, orientation = state.position_m, state.orientation_xyzw
    if sources.altitude == 'barometer':
        position = _vector((*position[:2], observation.sensors.barometer.altitude_m), 'pose.position_m')
    if sources.attitude == 'imu':
        orientation = normalize(observation.attitude_estimate.orientation_xyzw)
    return replace(state, position_m=position, orientation_xyzw=orientation)
