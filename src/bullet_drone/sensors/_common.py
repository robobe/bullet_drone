"""Measurement noise and source-state chronology shared by sensor models."""

from math import isfinite
from random import Random

from bullet_drone.contracts import LifecycleError
from bullet_drone.models import DroneState


def validate_state(state: DroneState, previous: DroneState | None = None) -> None:
    if (type(state.step_index) is not int or state.step_index < 0
            or not isfinite(state.simulation_time_s) or state.simulation_time_s < 0):
        raise ValueError("Sensor source step/time must be finite and nonnegative")
    if previous is not None:
        if state.episode_id != previous.episode_id or state.drone_id != previous.drone_id:
            raise LifecycleError("Sensor source identity changed; reset the sensor")
        if (state.step_index <= previous.step_index
                or state.simulation_time_s <= previous.simulation_time_s):
            raise ValueError("Sensor source step and time must advance")


def noisy(value: float, bias: float, stddev: float, rng: Random) -> float:
    result = value + bias + rng.gauss(0.0, stddev)
    if not isfinite(result):
        raise ValueError("Sensor measurement must be finite")
    return result
