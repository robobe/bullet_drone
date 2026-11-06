"""Own the physics/FDM/MSP loop; no imports from another example."""
from contextlib import ExitStack
from dataclasses import dataclass
from math import ceil
import time

from bullet_drone.betaflight.backend import BetaflightBackend
from bullet_drone.betaflight.bridge import UdpSitlBridge
from bullet_drone.config import ControlSettings, DroneSettings, SimulationConfig, SimulationSettings, _number
from bullet_drone.environment import DroneEnvironment
from bullet_drone.models import DroneObservation
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.sensors import BarometerSensor, ImuSensor
from bullet_drone.vehicle import load_racer_parameters
from examples.altitde_hold_bt.demo_cli import DEFAULT_SITL_BINARY
from examples.altitde_hold_bt.diagnostics import REPORT_INTERVAL_S, arming_blockers, log_status, log_transition
from examples.altitde_hold_bt.mission import AltitudeHoldMission, Phase
from examples.altitde_hold_bt.sitl import SAFE_CHANNELS, SitlSession


PHYSICS_HZ = 240
TIMESTEP_S = 1 / PHYSICS_HZ
DEFAULT_MAX_DURATION_S = 90.0
STATUS_PERIOD_S = 0.1
FEEDBACK_TIMEOUT_S = 0.5
MASS_KG = 1.0
GRAVITY_M_S2 = 9.81
GRAVITY_VECTOR_M_S2 = (0.0, 0.0, -GRAVITY_M_S2)
INITIAL_POSITION_M = (0.0, 0.0, 0.015)
CAMERA_DISTANCE_M = 5.0
CAMERA_YAW_DEG = 45
CAMERA_PITCH_DEG = -20
CAMERA_TARGET_M = (0.0, 0.0, 1.5)


@dataclass(frozen=True)
class DemoResult:
    observation: DroneObservation
    transitions: tuple


def run_demo(*, gui=True, sitl_binary=DEFAULT_SITL_BINARY,
             max_duration_s=DEFAULT_MAX_DURATION_S, on_observation=None):
    duration = _number(max_duration_s, 'max_duration_s', positive=True)
    config = SimulationConfig(
        SimulationSettings(TIMESTEP_S, ceil(duration / TIMESTEP_S), GRAVITY_VECTOR_M_S2, gui=gui),
        DroneSettings(MASS_KG, INITIAL_POSITION_M), control=ControlSettings('betaflight'))
    vehicle = load_racer_parameters()
    mission = AltitudeHoldMission(MASS_KG, GRAVITY_M_S2,
                                  len(vehicle.rotor_positions_m) * vehicle.max_thrust_per_rotor_n)
    session = SitlSession(sitl_binary)
    with ExitStack() as resources:
        physics = PyBulletPhysics(config, vehicle)
        resources.callback(physics.close)
        environment = DroneEnvironment(physics,
            imu=ImuSensor(GRAVITY_VECTOR_M_S2, config.sensors.imu, sample_period_s=TIMESTEP_S),
            barometer=BarometerSensor(config.sensors.barometer))
        resources.callback(environment.close)
        backend = BetaflightBackend(vehicle, UdpSitlBridge())
        resources.callback(backend.close)
        # Last acquired, first released: disarm and stop SITL before closing physics.
        sitl = resources.enter_context(session)
        observation = environment.reset()
        backend.reset(observation)
        if gui:
            import pybullet
            pybullet.resetDebugVisualizerCamera(CAMERA_DISTANCE_M, CAMERA_YAW_DEG,
                                               CAMERA_PITCH_DEG, CAMERA_TARGET_M)
        if on_observation is not None:
            on_observation(observation)
        started = time.monotonic()
        next_status_s = started
        next_fdm_s = started
        status = None
        channels = SAFE_CHANNELS
        next_report_s = 0.0
        last_blockers = None
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
            try:
                channels = mission.command(observation.ground_truth, status, TIMESTEP_S)
            except Exception:
                log_status(mission.phase, observation.ground_truth, status, channels, backend,
                           sitl.client, time.monotonic() - started)
                raise
            sitl.client.rc.update(channels)
            if mission.phase != previous_phase:
                log_transition(previous_phase, mission.phase, observation.ground_truth.simulation_time_s)
            elapsed_s = time.monotonic() - started
            if (elapsed_s >= next_report_s or status.arming_disabled_flags != last_blockers
                    or mission.phase != previous_phase):
                log_status(mission.phase, observation.ground_truth, status, channels, backend,
                           sitl.client, elapsed_s)
                next_report_s = elapsed_s + REPORT_INTERVAL_S
                last_blockers = status.arming_disabled_flags
            if mission.phase == Phase.COMPLETE:
                return DemoResult(observation, tuple(mission.transitions))
            if observation.ground_truth.simulation_time_s + TIMESTEP_S > duration:
                raise TimeoutError(f'Betaflight altitude hold timed out in {mission.phase}, '
                    f'armed={status.armed}, blockers={arming_blockers(status.arming_disabled_flags)}')
            remaining = next_fdm_s - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            command = backend.command(observation)
            # Catch-up bursts make upstream SITL's arrival-based clock run too fast.
            next_fdm_s = time.monotonic() + TIMESTEP_S
            observation = (environment.hold() if mission.phase in (Phase.PREARM, Phase.ARMING)
                           else environment.step(command))
            if on_observation is not None:
                on_observation(observation)
