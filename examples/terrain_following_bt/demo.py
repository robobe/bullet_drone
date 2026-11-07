"""Own the Betaflight FDM/MSP/PyBullet terrain-following loop."""

from contextlib import ExitStack
from dataclasses import dataclass
from math import ceil, radians
import time

from bullet_drone.betaflight.backend import BetaflightBackend
from bullet_drone.betaflight.bridge import UdpSitlBridge
from bullet_drone.config import (
    ControlSettings, DroneSettings, RangefinderSettings, SensorsSettings,
    SimulationConfig, SimulationSettings, TerrainSettings, _number,
)
from bullet_drone.controllers import AltitudeController, PidSettings, PositionController, VelocityController
from bullet_drone.controllers.internal import AttitudeController
from bullet_drone.environment import DroneEnvironment
from bullet_drone.models import DroneObservation
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.sensors import BarometerSensor, ImuSensor, RangefinderSensor
from bullet_drone.vehicle import load_racer_parameters
from examples.terrain_following_bt.cli import DEFAULT_SITL_BINARY, PROJECT_ROOT
from examples.terrain_following_bt.diagnostics import (
    REPORT_INTERVAL_S, arming_blockers, log_status, log_transition, log_waypoint,
)
from examples.terrain_following_bt.mission import (
    Phase, TerrainMission, Transition, WaypointArrival,
)
from examples.terrain_following_bt.sitl import SAFE_CHANNELS, SitlSession


PHYSICS_HZ = 240
TIMESTEP_S = 1 / PHYSICS_HZ
DEFAULT_MAX_DURATION_S = 180.0
STATUS_PERIOD_S = 0.1
FEEDBACK_TIMEOUT_S = 0.5
MASS_KG = 1.0
GRAVITY_M_S2 = 9.81
GRAVITY_VECTOR_M_S2 = (0.0, 0.0, -GRAVITY_M_S2)
INITIAL_POSITION_M = (0.0, 0.0, 0.015)
TERRAIN_HEIGHTMAP_PATH = PROJECT_ROOT / 'custom/terrain/terrain_heightmap.f32'
TERRAIN_WORLD_SIZE_M = 500.0
TERRAIN_SAMPLES_X = 121
TERRAIN_SAMPLES_Z = 121
TERRAIN_ORIGIN_M = (-250.0, -250.0)
RANGEFINDER_MAX_RANGE_M = 50.0
POSITION_KP = 0.8
MAX_NAVIGATION_SPEED_M_S = 3.0
VELOCITY_KP = 2.0
MAX_NAVIGATION_ACCELERATION_M_S2 = 1.5
CLEARANCE_POSITION_KP = 1.5
MAX_CLEARANCE_TARGET_RATE_M_S = 1.5
CLEARANCE_VELOCITY_KP = 6.0
CLEARANCE_VELOCITY_KI = 4.0
MAX_CLEARANCE_ACCELERATION_M_S2 = 3.0
MAX_CLEARANCE_INTEGRAL_ACCELERATION_M_S2 = 2.0
POSITION_PID = PidSettings(POSITION_KP, 0.0, 0.0, MAX_NAVIGATION_SPEED_M_S, 0.0)
VELOCITY_PID = PidSettings(VELOCITY_KP, 0.0, 0.0, MAX_NAVIGATION_ACCELERATION_M_S2, 0.0)
CLEARANCE_POSITION_PID = PidSettings(
    CLEARANCE_POSITION_KP, 0.0, 0.0, MAX_CLEARANCE_TARGET_RATE_M_S, 0.0)
CLEARANCE_VELOCITY_PID = PidSettings(
    CLEARANCE_VELOCITY_KP, CLEARANCE_VELOCITY_KI, 0.0,
    MAX_CLEARANCE_ACCELERATION_M_S2, MAX_CLEARANCE_INTEGRAL_ACCELERATION_M_S2)
MAX_TILT_RAD = radians(15)
ATTITUDE_KP_S_INV = (4.0, 4.0, 4.0)
MAX_BODY_RATES_RAD_S = (2.0, 2.0, 2.0)


@dataclass(frozen=True, slots=True)
class DemoResult:
    observation: DroneObservation
    transitions: tuple[Transition, ...]
    arrivals: tuple[WaypointArrival, ...]


def run_demo(*, sitl_binary=DEFAULT_SITL_BINARY, max_duration_s=DEFAULT_MAX_DURATION_S,
             on_observation=None):
    duration = _number(max_duration_s, 'max_duration_s', positive=True)
    if not TERRAIN_HEIGHTMAP_PATH.is_file():
        raise FileNotFoundError(f'Terrain heightmap is missing: {TERRAIN_HEIGHTMAP_PATH}')
    config = SimulationConfig(
        SimulationSettings(TIMESTEP_S, ceil(duration / TIMESTEP_S), GRAVITY_VECTOR_M_S2, gui=False),
        DroneSettings(MASS_KG, INITIAL_POSITION_M),
        control=ControlSettings('betaflight'),
        sensors=SensorsSettings(rangefinder=RangefinderSettings(
            enabled=True, max_range_m=RANGEFINDER_MAX_RANGE_M)),
        terrain=TerrainSettings(
            heightmap_path=str(TERRAIN_HEIGHTMAP_PATH), world_size_m=TERRAIN_WORLD_SIZE_M,
            samples_x=TERRAIN_SAMPLES_X, samples_z=TERRAIN_SAMPLES_Z, origin_m=TERRAIN_ORIGIN_M),
    )
    vehicle = load_racer_parameters()
    max_thrust = len(vehicle.rotor_positions_m) * vehicle.max_thrust_per_rotor_n
    mission = TerrainMission(
        PositionController(POSITION_PID),
        VelocityController(MASS_KG, max_thrust, VELOCITY_PID, MAX_TILT_RAD),
        AltitudeController(MASS_KG, GRAVITY_M_S2, max_thrust,
                           CLEARANCE_POSITION_PID, CLEARANCE_VELOCITY_PID),
        AttitudeController(ATTITUDE_KP_S_INV, MAX_BODY_RATES_RAD_S),
        max_thrust,
    )
    with ExitStack() as resources:
        physics = PyBulletPhysics(config, vehicle)
        resources.callback(physics.close)
        environment = DroneEnvironment(
            physics,
            imu=ImuSensor(GRAVITY_VECTOR_M_S2, config.sensors.imu, sample_period_s=TIMESTEP_S),
            barometer=BarometerSensor(config.sensors.barometer),
            rangefinder=RangefinderSensor(physics, config.sensors.rangefinder),
        )
        resources.callback(environment.close)
        backend = BetaflightBackend(vehicle, UdpSitlBridge())
        resources.callback(backend.close)
        sitl = resources.enter_context(SitlSession(sitl_binary))
        observation = environment.reset()
        backend.reset(observation)
        if on_observation is not None:
            on_observation(observation)
        mission.reset(observation)
        started = time.monotonic()
        next_status_s = started
        next_fdm_s = started
        next_report_s = 0.0
        last_blockers = None
        status = None
        channels = SAFE_CHANNELS
        log_transition(None, mission.phase, observation.ground_truth.simulation_time_s)
        while True:
            sitl.check_running()
            now = time.monotonic()
            if now >= next_status_s:
                status = sitl.commands.read_status()
                next_status_s = time.monotonic() + STATUS_PERIOD_S
            if status is None or not 0 <= time.monotonic() - status.received_at <= FEEDBACK_TIMEOUT_S:
                raise RuntimeError('Betaflight status is stale')
            previous_phase = mission.phase
            previous_index = mission.index
            arrival_count = len(mission.arrivals)
            try:
                channels = mission.command(observation, status, TIMESTEP_S, physics.terrain_contact())
            except Exception:
                log_status(mission.phase, mission.index, observation, mission.clearance_rate_m_s,
                           mission.heading_rad, status, channels, backend, sitl.client, time.monotonic() - started)
                raise
            sitl.client.rc.update(channels)
            if len(mission.arrivals) != arrival_count:
                for arrival in mission.arrivals[arrival_count:]:
                    log_waypoint(arrival, settled=False)
            if mission.phase != previous_phase:
                log_transition(previous_phase, mission.phase, observation.ground_truth.simulation_time_s)
            elapsed_s = time.monotonic() - started
            if (elapsed_s >= next_report_s or status.arming_disabled_flags != last_blockers
                    or mission.phase != previous_phase or mission.index != previous_index):
                log_status(mission.phase, mission.index, observation, mission.clearance_rate_m_s,
                           mission.heading_rad, status, channels, backend, sitl.client, elapsed_s)
                next_report_s = elapsed_s + REPORT_INTERVAL_S
                last_blockers = status.arming_disabled_flags
            if mission.phase == Phase.COMPLETE:
                return DemoResult(observation, tuple(mission.transitions), tuple(mission.arrivals))
            if observation.ground_truth.simulation_time_s + TIMESTEP_S > duration:
                raise TimeoutError(
                    f'Terrain-following flight timed out in {mission.phase}, '
                    f'armed={status.armed}, blockers={arming_blockers(status.arming_disabled_flags)}')
            remaining = next_fdm_s - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            rotor_command = backend.command(observation)
            next_fdm_s = time.monotonic() + TIMESTEP_S
            observation = (environment.hold() if mission.phase in (Phase.PREARM, Phase.ARMING)
                           else environment.step(rotor_command))
            if on_observation is not None:
                on_observation(observation)

