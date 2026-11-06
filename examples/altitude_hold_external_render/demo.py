"""Independent paced physics loop, with an exported Godot process as viewer."""
from contextlib import ExitStack
from dataclasses import dataclass
from math import ceil
import time

from loguru import logger

from bullet_drone.config import DroneSettings, SimulationConfig, SimulationSettings, _number
from bullet_drone.controllers import AltitudeController, InternalControlSettings, PidSettings
from bullet_drone.models import DroneObservation
from bullet_drone.rc import RcCodec
from bullet_drone.simulation import RcSimulation
from bullet_drone.vehicle import load_racer_parameters
from examples.altitude_hold_external_render.mission import AltitudeHoldMission, Phase, Transition
from examples.altitude_hold_external_render.renderer import ExternalRenderer


PHYSICS_HZ = 240
TIMESTEP_S = 1 / PHYSICS_HZ
MAX_DURATION_S = 45.0
MASS_KG = 1.0
GRAVITY_M_S2 = 9.81
WORLD_GRAVITY_M_S2 = (0.0, 0.0, -GRAVITY_M_S2)
INITIAL_POSITION_M = (0.0, 0.0, 0.015)
ATTITUDE_KP_S_INV = (4.0, 4.0, 4.0)
BODY_RATE_LIMITS_RAD_S = (2.0, 2.0, 2.0)
RATE_PID = PidSettings(kp=0.05, ki=0.0, kd=0.0, output_limit=0.1, integral_limit=0.0)
CONTROL_SETTINGS = InternalControlSettings(ATTITUDE_KP_S_INV, BODY_RATE_LIMITS_RAD_S, (RATE_PID,) * 3)
ALTITUDE_PID = PidSettings(kp=1.5, ki=0.0, kd=0.0, output_limit=1.0, integral_limit=0.0)
VERTICAL_SPEED_PID = PidSettings(kp=6.0, ki=4.0, kd=0.0, output_limit=3.0, integral_limit=2.0)


@dataclass(frozen=True, slots=True)
class DemoResult:
    observation: DroneObservation
    transitions: tuple[Transition, ...]


def run_demo(renderer, render_config, *, max_duration_s=MAX_DURATION_S, on_observation=None):
    duration = _number(max_duration_s, 'max_duration_s', positive=True)
    config = SimulationConfig(
        SimulationSettings(TIMESTEP_S, ceil(duration / TIMESTEP_S), WORLD_GRAVITY_M_S2),
        DroneSettings(MASS_KG, INITIAL_POSITION_M))
    vehicle = load_racer_parameters()
    max_thrust = len(vehicle.rotor_positions_m) * vehicle.max_thrust_per_rotor_n
    mission = AltitudeHoldMission(
        AltitudeController(MASS_KG, GRAVITY_M_S2, max_thrust, ALTITUDE_PID, VERTICAL_SPEED_PID),
        RcCodec(max_thrust, BODY_RATE_LIMITS_RAD_S))
    with ExitStack() as resources:
        viewer = resources.enter_context(ExternalRenderer(renderer, render_config))
        simulation = RcSimulation(config, CONTROL_SETTINGS, vehicle)
        resources.callback(simulation.close)
        observation = simulation.reset()
        viewer.wait_ready(observation)
        mission.reset(observation)
        viewer.set_state(observation, 'running', 'automatic_start')
        started = time.monotonic()
        if on_observation is not None:
            on_observation(observation)
        while True:
            viewer.ensure_running()
            previous_phase = mission.phase
            command = mission.command(observation, TIMESTEP_S)
            if mission.phase != previous_phase:
                logger.info('External-render altitude hold: {} at {:.3f} s',
                            mission.phase, observation.ground_truth.simulation_time_s)
            if mission.phase == Phase.COMPLETE:
                viewer.set_state(observation, 'stopped', 'completed')
                return DemoResult(observation, tuple(mission.transitions))
            if observation.ground_truth.simulation_time_s + TIMESTEP_S > duration:
                raise TimeoutError(f'Altitude hold timed out in {mission.phase} after {duration:g} s')
            observation = simulation.step(command)
            viewer.consume(observation)
            if on_observation is not None:
                on_observation(observation)
            remaining = started + observation.ground_truth.simulation_time_s - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
