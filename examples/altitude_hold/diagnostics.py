"""Human-readable altitude-hold state and observation output."""

from collections.abc import Sequence

import click

from bullet_drone.models import DroneObservation
from examples.altitude_hold.mission import Phase


PHASE_COLORS = {
    Phase.SETTLE: 'yellow',
    Phase.TAKEOFF: 'green',
    Phase.HOLD: 'cyan',
    Phase.LAND: 'yellow',
    Phase.DISARMED: 'red',
    Phase.COMPLETE: 'bright_green',
}


def print_phase_change(previous: Phase | None, current: Phase, simulation_time_s: float) -> None:
    change = f'{previous.value} -> {current.value}' if previous else current.value
    click.secho(f'[STATE {simulation_time_s:7.2f}s] {change}',
                fg=PHASE_COLORS[current], bold=True)


def _vector(values: Sequence[float], precision: int = 3) -> str:
    return '(' + ', '.join(f'{value:.{precision}f}' for value in values) + ')'


def print_observation(observation: DroneObservation, phase: Phase) -> None:
    state = observation.ground_truth
    sensors = observation.sensors
    imu = sensors.imu
    barometer = sensors.barometer
    rangefinder = sensors.rangefinder
    attitude = observation.attitude_estimate

    title = click.style(
        f'[OBS {state.simulation_time_s:7.2f}s | step {state.step_index:6d} | '
        f'{phase.value.upper()}]', fg='bright_cyan', bold=True,
    )
    click.echo(
        f'{title} drone={state.drone_id} episode={state.episode_id}\n'
        f'  GT position_m={_vector(state.position_m)} '
        f'orientation_xyzw={_vector(state.orientation_xyzw)}\n'
        f'     velocity_m_s={_vector(state.linear_velocity_m_s)} '
        f'angular_velocity_rad_s={_vector(state.angular_velocity_rad_s)}\n'
        f'     rotor_speeds_rad_s={_vector(state.rotor_speeds_rad_s, 1)}\n'
        f'  IMU specific_force_m_s2='
        f'{None if imu is None else _vector(imu.specific_force_m_s2)} '
        f'angular_velocity_rad_s='
        f'{None if imu is None else _vector(imu.angular_velocity_rad_s)}\n'
        f'  BARO pressure_pa={None if barometer is None else f"{barometer.pressure_pa:.2f}"} '
        f'altitude_m={None if barometer is None else f"{barometer.altitude_m:.3f}"}\n'
        f'  RANGE distance_cm={None if rangefinder is None else rangefinder.distance_cm} '
        f'valid={None if rangefinder is None else rangefinder.valid}\n'
        f'  ATTITUDE_ESTIMATE_xyzw='
        f'{None if attitude is None else _vector(attitude.orientation_xyzw)}',
    )
