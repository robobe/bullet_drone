"""Colored Betaflight Acro square diagnostics without sink configuration."""

import time

from loguru import logger

from examples.position_square_acro.mission import Phase


ARMING_FLAG_NAMES = (
    'NOGYRO', 'FAILSAFE', 'RXLOSS', 'NOT_DISARMED', 'BOXFAILSAFE', 'RUNAWAY',
    'CRASH', 'THROTTLE', 'ANGLE', 'BOOTGRACE', 'NOPREARM', 'LOAD', 'CALIB',
    'CLI', 'CMS', 'BST', 'MSP', 'PARALYZE', 'GPS', 'RESCUE_SW', 'DSHOT_TELEM',
    'REBOOT_REQD', 'DSHOT_BBANG', 'NO_ACC_CAL', 'MOTOR_PROTO', 'FLIP_SWITCH',
    'ALT_HOLD_SW', 'POS_HOLD_SW', 'AUTOPILOT_SW', 'ARM_SWITCH',
)
KNOWN_ARMING_MASK = (1 << len(ARMING_FLAG_NAMES)) - 1
REPORT_INTERVAL_S = 1.0
ARM_CHANNEL_INDEX = 4
THROTTLE_CHANNEL_INDEX = 2
PHASE_COLORS = {
    Phase.PREARM: 'yellow', Phase.ARMING: 'magenta', Phase.TAKEOFF: 'cyan',
    Phase.SQUARE: 'magenta', Phase.HOLD: 'green', Phase.LAND: 'blue',
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
        f'ACRO STATE {{}} -> <{color}>{{}}</{color}> | sim={{:.3f}}s',
        'STARTUP' if previous is None else previous.name, phase.name, simulation_time_s)


def log_waypoint(arrival, *, settled):
    color = 'green' if settled else 'cyan'
    logger.opt(colors=True).info(
        f'ACRO <{color}>waypoint {{}} {{}}</{color}> | target={{}} m | sim={{:.3f}}s',
        arrival.index, 'settled' if settled else 'active', arrival.target_position_m,
        arrival.state.simulation_time_s)


def log_status(phase, waypoint_index, state, status, channels, backend, client, elapsed_s):
    now = time.monotonic()
    motor_age = 'NONE' if backend.received_at is None else f'{now - backend.received_at:.3f}s'
    motors = ', '.join(f'{value:.3f}' for value in backend.latest)
    factor = state.simulation_time_s / elapsed_s if elapsed_s > 0 else 0.0
    logger.info(
        'STATUS phase={} waypoint={} sim={:.3f}s wall={:.3f}s RTF={:.2f}x armed={} '
        'blockers={} (0x{:08x}) | roll={} pitch={} throttle={} yaw={} arm={} RC_sent={} | '
        'XYZ=({:.3f},{:.3f},{:.3f})m motors=({}) motor_age={} MSP_age={:.3f}s',
        phase.name, waypoint_index, state.simulation_time_s, elapsed_s, factor,
        status.armed, arming_blockers(status.arming_disabled_flags), status.arming_disabled_flags,
        *channels[:5], client.rc.sent, *state.position_m, motors, motor_age,
        now - status.received_at)
