"""Independent paced physics loop, with an exported Godot process as viewer."""
from contextlib import ExitStack
from dataclasses import replace
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
    CONTROL_SETTINGS, ALTITUDE_PID, VERTICAL_SPEED_PID, BODY_RATE_LIMITS_RAD_S,
)
from examples.wind_control.service import SimulationService, DEFAULT_SETUP_TAKEOFF_HEIGHT_M

SETUP_POLL_INTERVAL_S = 0.01
SPAWN_CLEARANCE_M = 0.20


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
        simulation = RcSimulation(config, CONTROL_SETTINGS, vehicle)
        resources.callback(simulation.close)
        observation = simulation.reset()
        viewer.wait_ready(observation)
        service = resources.enter_context(SimulationService(viewer.transport))
        def replace_spawn(request):
            nonlocal simulation, config
            candidate_config = replace(config, drone=replace(config.drone,
                initial_position_m=request.position_m))
            candidate = RcSimulation(candidate_config, CONTROL_SETTINGS, vehicle)
            try:
                candidate_observation = candidate.reset()
                if candidate.terrain_clearance_m() < SPAWN_CLEARANCE_M:
                    raise ValueError('terrain_clearance')
                candidate.set_wind(simulation.wind.settings)
            except BaseException:
                candidate.close()
                raise
            old = simulation
            simulation, config = candidate, candidate_config
            resources.callback(candidate.close)
            old.close()
            return candidate_observation

        started = None

        def prepare_mission(current):
            mission.target_altitude_m = config.drone.initial_position_m[2] + DEFAULT_SETUP_TAKEOFF_HEIGHT_M
            mission.reset(current)

        def start_mission(current):
            nonlocal started
            prepare_mission(current)
            started = time.monotonic() - current.ground_truth.simulation_time_s

        def reset_simulation():
            from bullet_simulation_protocol.requests import SpawnPayload
            current = replace_spawn(SpawnPayload(config.drone.initial_position_m))
            prepare_mission(current)
            return current

        logger.info('Stopped setup: select a spawn in Godot, then press Start')
        while True:
            viewer.ensure_running()
            observation = service.update(simulation, observation, replace_spawn=replace_spawn,
                reset_simulation=reset_simulation, start_mission=start_mission) or observation
            viewer.set_running(service.running)
            if not service.running:
                viewer.consume(observation)
                time.sleep(SETUP_POLL_INTERVAL_S)
                continue
            previous_phase = mission.phase
            command = mission.command(observation, TIMESTEP_S)
            if mission.phase != previous_phase:
                logger.info('External-render altitude hold: {} at {:.3f} s',
                            mission.phase, observation.ground_truth.simulation_time_s)
            if mission.phase == Phase.COMPLETE:
                service.complete()
                continue
            if observation.ground_truth.simulation_time_s + TIMESTEP_S > duration:
                service.running = False
                service.ready_to_start = False
                service.reason = 'mission_timeout'
                continue
            observation = simulation.step(command)
            viewer.consume(observation)
            if on_observation is not None:
                on_observation(observation)
            remaining = started + observation.ground_truth.simulation_time_s - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
