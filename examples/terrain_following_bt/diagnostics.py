"""Colored terrain mission diagnostics without sink configuration."""

import time
from math import degrees

from loguru import logger
from bullet_drone.geometry import yaw_from_quaternion

from examples.terrain_following_bt.mission import Phase


ARMING_FLAG_NAMES = (
    'NOGYRO', 'FAILSAFE', 'RXLOSS', 'NOT_DISARMED', 'BOXFAILSAFE', 'RUNAWAY',
    'CRASH', 'THROTTLE', 'ANGLE', 'BOOTGRACE', 'NOPREARM', 'LOAD', 'CALIB',
    'CLI', 'CMS', 'BST', 'MSP', 'PARALYZE', 'GPS', 'RESCUE_SW', 'DSHOT_TELEM',
    'REBOOT_REQD', 'DSHOT_BBANG', 'NO_ACC_CAL', 'MOTOR_PROTO', 'FLIP_SWITCH',
    'ALT_HOLD_SW', 'POS_HOLD_SW', 'AUTOPILOT_SW', 'ARM_SWITCH',
)
KNOWN_ARMING_MASK = (1 << len(ARMING_FLAG_NAMES)) - 1
REPORT_INTERVAL_S = 1.0
PHASE_COLORS = {
    Phase.PREARM: 'yellow', Phase.ARMING: 'magenta', Phase.TAKEOFF: 'cyan',
    Phase.OUTBOUND: 'magenta', Phase.RETURN: 'blue', Phase.LAND: 'blue',
    Phase.DISARMED: 'yellow', Phase.COMPLETE: 'green',
}


def arming_blockers(mask):
    names = [name for bit, name in enumerate(ARMING_FLAG_NAMES) if mask & (1 << bit)]
    unknown = mask & ~KNOWN_ARMING_MASK
    if unknown:
        names.append(f'UNKNOWN(0x{unknown:08x})')
    return ','.join(names) if names else 'NONE'


def log_transition(previous, phase, simulation_time_s):
    color = PHASE_COLORS[phase]
    logger.opt(colors=True).info(
        f'TERRAIN STATE {{}} -> <{color}>{{}}</{color}> | sim={{:.3f}}s',
        'STARTUP' if previous is None else previous.name, phase.name, simulation_time_s)


def log_waypoint(arrival, *, settled):
    color = 'green' if settled else 'cyan'
    logger.opt(colors=True).info(
        f'TERRAIN <{color}>waypoint {{}} {{}}</{color}> | phase={{}} target_xy={{}}m '
        'clearance={:.2f}m sim={:.3f}s',
        arrival.index, 'settled' if settled else 'passed', arrival.phase.name,
        arrival.target_xy_m, arrival.clearance_m, arrival.state.simulation_time_s)


def log_status(phase, waypoint_index, observation, clearance_rate_m_s, target_yaw_rad,
               status, channels, backend, client, elapsed_s):
    now = time.monotonic()
    state = observation.ground_truth
    reading = observation.sensors.rangefinder
    clearance = 'INVALID' if reading is None or reading.distance_cm is None else f'{reading.distance_cm / 100:.2f}m'
    motor_age = 'NONE' if backend.received_at is None else f'{now - backend.received_at:.3f}s'
    motors = ', '.join(f'{value:.3f}' for value in backend.latest)
    factor = state.simulation_time_s / elapsed_s if elapsed_s > 0 else 0.0
    logger.info(
        'STATUS phase={} waypoint={} sim={:.2f}s wall={:.2f}s RTF={:.2f}x armed={} '
        'blockers={} | RC(throttle={},arm={}) | XY=({:.2f},{:.2f})m Z={:.2f}m '
        'yaw={:.1f}° target={:.1f}° '
        'clearance={} clearance_rate={:.2f}m/s | motors=({}) motor_age={} MSP_age={:.3f}s',
        phase.name, waypoint_index, state.simulation_time_s, elapsed_s, factor,
        status.armed, arming_blockers(status.arming_disabled_flags), channels[2], channels[4],
        state.position_m[0], state.position_m[1], state.position_m[2],
        degrees(yaw_from_quaternion(state.orientation_xyzw)), degrees(target_yaw_rad), clearance,
        clearance_rate_m_s, motors, motor_age, now - status.received_at)
