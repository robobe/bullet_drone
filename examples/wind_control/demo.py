"""Independent paced physics loop, with an exported Godot process as viewer."""
from contextlib import ExitStack
from math import ceil
import time

from loguru import logger

from bullet_drone.config import DroneSettings, SimulationConfig, SimulationSettings, _number
from bullet_drone.controllers import AltitudeController
from bullet_drone.rc import RcCodec
from bullet_drone.simulation import RcSimulation
from bullet_drone.vehicle import load_racer_parameters
from examples.altitude_hold_external_render.mission import AltitudeHoldMission, Phase
from external_render.renderer import ExternalRenderer


from examples.altitude_hold_external_render.demo import (
    MAX_DURATION_S, TIMESTEP_S, GRAVITY_M_S2, WORLD_GRAVITY_M_S2, MASS_KG, INITIAL_POSITION_M,
    CONTROL_SETTINGS, ALTITUDE_PID, VERTICAL_SPEED_PID, BODY_RATE_LIMITS_RAD_S, DemoResult,
)
from examples.wind_control.panel import WindPanel


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
        viewer = resources.enter_context(ExternalRenderer(renderer, render_config, profile_name='wind_control'))
        panel = WindPanel()
        resources.callback(panel.close)
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
            panel.update(simulation, viewer, observation)
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
