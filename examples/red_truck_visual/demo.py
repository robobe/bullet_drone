"""Own the rendered visual-servo flight and Betaflight Acro loop."""
from contextlib import ExitStack
from dataclasses import dataclass
from math import ceil
import time

from loguru import logger

from bullet_drone.betaflight.backend import BetaflightBackend
from examples.red_truck_visual.timing import PacedSitlBridge
from bullet_drone.config import ControlSettings, DroneSettings, SimulationConfig, SimulationSettings, _number
from bullet_drone.controllers import AltitudeController, PidSettings
from bullet_drone.controllers.internal import AttitudeController
from bullet_drone.environment import DroneEnvironment
from bullet_drone.models import DroneObservation
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.rc import RcCodec
from bullet_drone.sensors import BarometerSensor, ImuSensor
from bullet_drone.vehicle import load_racer_parameters
from examples.red_truck_visual.demo_cli import DEFAULT_RENDER_CONFIG, DEFAULT_RENDERER, DEFAULT_SITL_BINARY
from examples.red_truck_visual.mission import (
    ATTITUDE_KP_S_INV, MAX_BODY_RATES_RAD_S, ALTITUDE_POSITION_PID, ALTITUDE_VELOCITY_PID,
    Phase, RedTruckMission,
)
from examples.red_truck_visual.renderer import ExternalRenderer
from examples.red_truck_visual.vision import VisionWorker
from examples.red_truck_visual.status import StatusWorker
from examples.red_truck_visual.diagnostics import Diagnostics
from examples.position_square_acro.sitl import SitlSession


PHYSICS_HZ = 240
TIMESTEP_S = 1 / PHYSICS_HZ
MAX_DURATION_S = 210.0
FEEDBACK_TIMEOUT_S = 0.5
VISION_TIMEOUT_STEPS = 120
CONTACT_MAX_AGE_STEPS = 240
MASS_KG = 1.0
GRAVITY_M_S2 = 9.81
GRAVITY_VECTOR_M_S2 = (0.0, 0.0, -GRAVITY_M_S2)
INITIAL_POSITION_M = (0.0, 0.0, 0.015)


@dataclass(frozen=True, slots=True)
class DemoResult:
    observation: DroneObservation
    transitions: tuple


def run_demo(*, renderer=DEFAULT_RENDERER, render_config=None, sitl_binary=DEFAULT_SITL_BINARY,
             max_duration_s=MAX_DURATION_S):
    max_duration_s = _number(max_duration_s, 'max_duration_s', positive=True)
    render_config = render_config or DEFAULT_RENDER_CONFIG
    max_steps = ceil(max_duration_s / TIMESTEP_S)
    config = SimulationConfig(
        SimulationSettings(TIMESTEP_S, max_steps, GRAVITY_VECTOR_M_S2, gui=False),
        DroneSettings(MASS_KG, INITIAL_POSITION_M), control=ControlSettings('betaflight'))
    vehicle = load_racer_parameters()
    max_thrust = len(vehicle.rotor_positions_m) * vehicle.max_thrust_per_rotor_n
    with ExitStack() as resources:
        viewer = resources.enter_context(ExternalRenderer(renderer, render_config))
        camera = viewer.publisher.config['camera']
        mission = RedTruckMission(
            AltitudeController(MASS_KG, GRAVITY_M_S2, max_thrust,
                               ALTITUDE_POSITION_PID, ALTITUDE_VELOCITY_PID),
            AttitudeController(ATTITUDE_KP_S_INV, MAX_BODY_RATES_RAD_S),
            RcCodec(max_thrust, MAX_BODY_RATES_RAD_S),
            max_thrust, camera_fov_x_deg=camera['fov_x_deg'],
            camera_aspect=camera['width_px'] / camera['height_px'])
        physics = PyBulletPhysics(config, vehicle)
        resources.callback(physics.close)
        environment = DroneEnvironment(
            physics, imu=ImuSensor(GRAVITY_VECTOR_M_S2, config.sensors.imu, sample_period_s=TIMESTEP_S),
            barometer=BarometerSensor(config.sensors.barometer))
        resources.callback(environment.close)
        bridge = PacedSitlBridge(TIMESTEP_S)
        backend = BetaflightBackend(vehicle, bridge)
        resources.callback(backend.close)
        sitl = resources.enter_context(SitlSession(sitl_binary))
        observation = environment.reset()
        backend.reset(observation)
        viewer.wait_ready(observation)
        vision = resources.enter_context(VisionWorker(viewer))
        statuses = resources.enter_context(StatusWorker(sitl.commands))
        mission.reset(observation)
        viewer.set_running(True)
        diagnostics = Diagnostics()

        for _ in range(max_steps):
            viewer.ensure_running()
            status = statuses.read()
            if status is None or not 0 <= time.monotonic() - status.received_at <= FEEDBACK_TIMEOUT_S:
                raise RuntimeError('Betaflight status is stale')
            state = observation.ground_truth
            viewer.consume(observation)
            latest_target = vision.target(state.step_index, VISION_TIMEOUT_STEPS)
            contact = viewer.poll_contact(stream_id=viewer.publisher.stream,
                current_step=state.step_index, max_age_steps=CONTACT_MAX_AGE_STEPS)
            try:
                channels = mission.command(observation, status, TIMESTEP_S,
                                           latest_target, contact=contact)
            except RuntimeError as error:
                if status.armed:
                    raise
                client = sitl.client
                rc_age = (None if client.rc.last_sent_at is None
                          else time.monotonic() - client.rc.last_sent_at)
                raise RuntimeError(
                    f'{error}; MSP connected={client.connected.is_set()}, error={client.error!r}, '
                    f'RC frames={client.rc.sent}, last_rc_age_s={rc_age}') from error
            sitl.client.rc.update(channels)
            diagnostics.report(mission, state, latest_target, channels)
            diagnostics.report_rates(state, bridge.sent, viewer.publisher.sequence, vision)
            if mission.phase == Phase.COMPLETE:
                viewer.set_running(False)
                logger.info('Red truck visual approach completed at step {}', state.step_index)
                return DemoResult(observation, tuple(mission.transitions))
            if state.simulation_time_s + TIMESTEP_S > max_duration_s:
                raise TimeoutError(f'Red truck visual approach timed out in {mission.phase}')
            rotor_command = backend.command(observation)
            observation = (environment.hold() if mission.phase in (Phase.PREARM, Phase.ARMING)
                           else environment.step(rotor_command))
        raise TimeoutError('Red truck visual approach exceeded simulation limit')
