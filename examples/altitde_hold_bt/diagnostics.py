"""Human-readable example diagnostics; no sinks or simulation-data storage."""
import time

from loguru import logger

from examples.altitde_hold_bt.mission import Phase


# Bit order from the pinned 2026.6.2 fc/runtime_config.c, not generic MSP flags.
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
WORLD_Z_AXIS = 2
PHASE_COLORS = {
    Phase.PREARM: 'yellow', Phase.ARMING: 'magenta', Phase.TAKEOFF: 'cyan',
    Phase.HOLD: 'green', Phase.LAND: 'blue', Phase.DISARMED: 'yellow',
    Phase.COMPLETE: 'green',
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
        f'STATE {{}} -> <{color}>{{}}</{color}> | sim={{:.3f}}s',
        'STARTUP' if previous is None else previous.name, phase.name, simulation_time_s)


def log_status(phase, state, status, channels, backend, client, elapsed_s):
    now = time.monotonic()
    motor_age = 'NONE' if backend.received_at is None else f'{now - backend.received_at:.3f}s'
    motors = ', '.join(f'{value:.3f}' for value in backend.latest)
    factor = state.simulation_time_s / elapsed_s if elapsed_s > 0 else 0.0
    logger.info(
        'STATUS phase={} sim={:.3f}s wall={:.3f}s RTF={:.2f}x armed={} blockers={} (0x{:08x}) | '
        'arm_pwm={} throttle_pwm={} RC_sent={} | Z={:.3f}m Vz={:.3f}m/s | '
        'motors=({}) motor_age={} MSP_age={:.3f}s',
        phase.name, state.simulation_time_s, elapsed_s, factor, status.armed,
        arming_blockers(status.arming_disabled_flags), status.arming_disabled_flags,
        channels[ARM_CHANNEL_INDEX], channels[THROTTLE_CHANNEL_INDEX], client.rc.sent,
        state.position_m[WORLD_Z_AXIS], state.linear_velocity_m_s[WORLD_Z_AXIS], motors, motor_age,
        now - status.received_at)
