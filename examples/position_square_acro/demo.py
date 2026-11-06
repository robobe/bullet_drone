"""Own the Betaflight FDM/MSP/PyBullet loop for the Acro square."""

from contextlib import ExitStack
from dataclasses import dataclass
from math import ceil, radians
import time

from bullet_drone.betaflight.backend import BetaflightBackend
from bullet_drone.betaflight.bridge import UdpSitlBridge
from bullet_drone.config import ControlSettings, DroneSettings, SimulationConfig, SimulationSettings, _number
from bullet_drone.controllers import AltitudeController, PidSettings, PositionController, VelocityController
from bullet_drone.controllers.internal import AttitudeController
from bullet_drone.environment import DroneEnvironment
from bullet_drone.models import DroneObservation
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.sensors import BarometerSensor, ImuSensor
from bullet_drone.vehicle import load_racer_parameters
from examples.position_square_acro.demo_cli import DEFAULT_SITL_BINARY
from examples.position_square_acro.diagnostics import (
    REPORT_INTERVAL_S, arming_blockers, log_status, log_transition, log_waypoint,
)
from examples.position_square_acro.mission import Phase, SquareMission, Transition, WaypointArrival
from examples.position_square_acro.sitl import SAFE_CHANNELS, SitlSession


PHYSICS_HZ = 240
TIMESTEP_S = 1 / PHYSICS_HZ
DEFAULT_MAX_DURATION_S = 120.0
STATUS_PERIOD_S = 0.1
FEEDBACK_TIMEOUT_S = 0.5
MASS_KG = 1.0
GRAVITY_M_S2 = 9.81
GRAVITY_VECTOR_M_S2 = (0.0, 0.0, -GRAVITY_M_S2)
INITIAL_POSITION_M = (0.0, 0.0, 0.015)
XY_POSITION_PID = PidSettings(0.8, 0.0, 0.0, 1.0, 0.0)
XY_VELOCITY_PID = PidSettings(2.0, 0.0, 0.0, 1.5, 0.0)
ALTITUDE_POSITION_PID = PidSettings(1.5, 0.0, 0.0, 1.0, 0.0)
ALTITUDE_VELOCITY_PID = PidSettings(6.0, 4.0, 0.0, 3.0, 2.0)
MAX_TILT_RAD = radians(15)
ATTITUDE_KP_S_INV = (4.0, 4.0, 4.0)
MAX_BODY_RATES_RAD_S = (2.0, 2.0, 2.0)


@dataclass(frozen=True, slots=True)
class DemoResult:
    observation: DroneObservation
    transitions: tuple[Transition, ...]
    arrivals: tuple[WaypointArrival, ...]


def run_demo(*, sitl_binary=DEFAULT_SITL_BINARY,
             max_duration_s=DEFAULT_MAX_DURATION_S, on_observation=None):
    duration = _number(max_duration_s, 'max_duration_s', positive=True)
    config = SimulationConfig(
        SimulationSettings(TIMESTEP_S, ceil(duration / TIMESTEP_S), GRAVITY_VECTOR_M_S2, gui=False),
        DroneSettings(MASS_KG, INITIAL_POSITION_M), control=ControlSettings('betaflight'))
    vehicle = load_racer_parameters()
    max_thrust = len(vehicle.rotor_positions_m) * vehicle.max_thrust_per_rotor_n
    mission = SquareMission(
        PositionController(XY_POSITION_PID),
        VelocityController(MASS_KG, max_thrust, XY_VELOCITY_PID, MAX_TILT_RAD),
        AltitudeController(MASS_KG, GRAVITY_M_S2, max_thrust,
                           ALTITUDE_POSITION_PID, ALTITUDE_VELOCITY_PID),
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
                channels = mission.command(observation, status, TIMESTEP_S)
            except Exception:
                log_status(mission.phase, mission.index, observation.ground_truth, status,
                           channels, backend, sitl.client, time.monotonic() - started)
                raise
            sitl.client.rc.update(channels)
            if len(mission.arrivals) != arrival_count:
                log_waypoint(mission.arrivals[-1], settled=True)
            if mission.phase != previous_phase:
                log_transition(previous_phase, mission.phase,
                               observation.ground_truth.simulation_time_s)
            if (mission.phase in (Phase.TAKEOFF, Phase.SQUARE)
                    and (mission.index != previous_index or mission.phase != previous_phase)):
                log_waypoint(WaypointArrival(mission.index, mission.waypoints[mission.index],
                                             observation.ground_truth), settled=False)
            elapsed_s = time.monotonic() - started
            if (elapsed_s >= next_report_s or status.arming_disabled_flags != last_blockers
                    or mission.phase != previous_phase or mission.index != previous_index):
                log_status(mission.phase, mission.index, observation.ground_truth, status,
                           channels, backend, sitl.client, elapsed_s)
                next_report_s = elapsed_s + REPORT_INTERVAL_S
                last_blockers = status.arming_disabled_flags
            if mission.phase == Phase.COMPLETE:
                return DemoResult(observation, tuple(mission.transitions), tuple(mission.arrivals))
            if observation.ground_truth.simulation_time_s + TIMESTEP_S > duration:
                raise TimeoutError(
                    f'Betaflight Acro square timed out in {mission.phase}, '
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
