"""Compose square-flight controllers and own the execution loop."""

from collections.abc import Callable
from dataclasses import dataclass
from math import ceil, radians
import time

from loguru import logger

from bullet_drone.config import DroneSettings, SimulationConfig, SimulationSettings, _number
from bullet_drone.controllers import AltitudeController, InternalControlSettings, PidSettings, PositionController, VelocityController
from bullet_drone.models import DroneObservation
from bullet_drone.rc import RcCodec
from bullet_drone.simulation import RcSimulation
from bullet_drone.vehicle import load_racer_parameters
from examples.position_square.mission import Phase, SquareMission, Transition, WaypointArrival
from examples.timing import RealtimeFactorLogger


SIMULATION_RATE_HZ = 240
TIMESTEP_S = 1 / SIMULATION_RATE_HZ
DEFAULT_MAX_DURATION_S = 90.0
DRONE_MASS_KG = 1.0
GRAVITY_M_S2 = 9.81
WORLD_GRAVITY_M_S2 = (0.0, 0.0, -GRAVITY_M_S2)
INITIAL_POSITION_M = (0.0, 0.0, 0.015)
ATTITUDE_KP_S_INV = (4.0, 4.0, 4.0)
MAX_BODY_RATES_RAD_S = (2.0, 2.0, 2.0)
BODY_RATE_PID = PidSettings(kp=0.05, ki=0.0, kd=0.0, output_limit=0.1, integral_limit=0.0)
FLIGHT_CONTROL_SETTINGS = InternalControlSettings(ATTITUDE_KP_S_INV, MAX_BODY_RATES_RAD_S,
                                                 (BODY_RATE_PID,) * len(ATTITUDE_KP_S_INV))
XY_POSITION_PID = PidSettings(kp=0.8, ki=0.0, kd=0.0, output_limit=1.0, integral_limit=0.0)
XY_VELOCITY_PID = PidSettings(kp=2.0, ki=0.0, kd=0.0, output_limit=1.5, integral_limit=0.0)
ALTITUDE_POSITION_PID = PidSettings(kp=1.5, ki=0.0, kd=0.0, output_limit=1.0, integral_limit=0.0)
# Ground-effect lift needs more downward integral authority during touchdown.
ALTITUDE_VELOCITY_PID = PidSettings(kp=6.0, ki=4.0, kd=0.0, output_limit=3.0, integral_limit=2.0)
MAX_TILT_RAD = radians(15)


class DemoTimeoutError(RuntimeError):
    """The square mission did not finish within its simulation-time budget."""


@dataclass(frozen=True, slots=True)
class DemoResult:
    observation: DroneObservation
    transitions: tuple[Transition, ...]
    arrivals: tuple[WaypointArrival, ...]


def run_demo(*, gui: bool = True, max_duration_s: float = DEFAULT_MAX_DURATION_S,
             on_observation: Callable[[DroneObservation], None] | None = None,
             pace: bool = False, control=None) -> DemoResult:
    duration = _number(max_duration_s, 'max_duration_s', positive=True)
    max_steps = ceil(duration / TIMESTEP_S)
    config = SimulationConfig(SimulationSettings(TIMESTEP_S, max_steps, WORLD_GRAVITY_M_S2, gui=gui),
                              DroneSettings(DRONE_MASS_KG, INITIAL_POSITION_M))
    vehicle = load_racer_parameters()
    max_thrust = len(vehicle.rotor_positions_m) * vehicle.max_thrust_per_rotor_n
    mission = SquareMission(
        PositionController(XY_POSITION_PID),
        VelocityController(DRONE_MASS_KG, max_thrust, XY_VELOCITY_PID, MAX_TILT_RAD),
        AltitudeController(DRONE_MASS_KG, GRAVITY_M_S2, max_thrust, ALTITUDE_POSITION_PID, ALTITUDE_VELOCITY_PID),
        RcCodec(max_thrust, FLIGHT_CONTROL_SETTINGS.max_rates_rad_s, config.rc),
    )
    simulation = RcSimulation(config, FLIGHT_CONTROL_SETTINGS, vehicle)
    try:
        observation = simulation.reset()
        if on_observation is not None:
            on_observation(observation)
        if gui:
            from examples.position_square.gui import frame_flight

            frame_flight()
        mission.reset(observation)
        started = time.monotonic()
        timing = RealtimeFactorLogger(observation.ground_truth.simulation_time_s, started,
                                      clock=time.monotonic)
        logger.info('Position square: settle at {:.3f} s', observation.ground_truth.simulation_time_s)
        step = 0
        if control is not None:
            control.transition('stopped', 'startup', observation)
        while True:
            if control is not None:
                action = control.poll()
                if action == 'reset' or (action == 'start' and control.state != 'running'):
                    try:
                        observation = simulation.reset()
                        mission.reset(observation)
                        step = 0
                        started = time.monotonic()
                        timing = RealtimeFactorLogger(observation.ground_truth.simulation_time_s, started,
                                                      clock=time.monotonic)
                        if on_observation is not None:
                            on_observation(observation)
                        control.transition('running' if action == 'start' else 'stopped',
                                           'started' if action == 'start' else 'reset', observation)
                    except Exception:
                        logger.exception('Simulation reset failed')
                        control.transition('error', 'reset_failed', observation)
                elif action in ('stop', 'controller_timeout', 'controller_changed'):
                    if control.state == 'running':
                        control.transition('stopped', action, observation)
                control.publish(observation)
                if control.state != 'running':
                    continue
            try:
                previous_phase, arrival_count = mission.phase, len(mission.arrivals)
                rc_command = mission.command(observation, simulation.timestep_s)
                if len(mission.arrivals) != arrival_count:
                    arrival = mission.arrivals[-1]
                    logger.info('Position square: waypoint {} settled at {:.3f} s', arrival.index,
                                arrival.state.simulation_time_s)
                if mission.phase != previous_phase:
                    logger.info('Position square: {} at {:.3f} s', mission.phase, observation.ground_truth.simulation_time_s)
                if mission.phase == Phase.COMPLETE:
                    if control is not None:
                        control.transition('stopped', 'completed', observation)
                        continue
                    break
                if (step + 1) * simulation.timestep_s > duration:
                    raise DemoTimeoutError(f'Position square timed out after {duration:g} s in phase {mission.phase}')
                observation = simulation.step(rc_command)
                if on_observation is not None:
                    on_observation(observation)
                if gui or pace:
                    remaining = started + observation.ground_truth.simulation_time_s - time.monotonic()
                    if remaining > 0:
                        time.sleep(remaining)
                timing.update(observation.ground_truth.simulation_time_s)
                step += 1
            except Exception:
                if control is None:
                    raise
                logger.exception('Simulation episode failed')
                control.transition('error', 'runtime_failure', observation)
        timing.update(observation.ground_truth.simulation_time_s, final=True)
        result = DemoResult(observation, tuple(mission.transitions), tuple(mission.arrivals))
    except BaseException:
        try:
            simulation.close()
        except BaseException:
            logger.exception('Simulation cleanup failed while handling a failure')
        raise
    else:
        simulation.close()
        return result
