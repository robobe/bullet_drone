"""Terminal diagnostics for square progress, without configuring logging sinks."""

from loguru import logger

from examples.position_square.mission import Phase


PHASE_COLORS = {
    Phase.SETTLE: 'yellow',
    Phase.TAKEOFF: 'cyan',
    Phase.SQUARE: 'magenta',
    Phase.HOLD: 'green',
    Phase.LAND: 'blue',
    Phase.DISARMED: 'yellow',
    Phase.COMPLETE: 'green',
}
ACTIVE_WAYPOINT_COLOR = 'cyan'
SETTLED_WAYPOINT_COLOR = 'green'


def log_transition(previous, phase, simulation_time_s):
    color = PHASE_COLORS[phase]
    logger.opt(colors=True).info(
        f'Position square: STATE {{}} -> <{color}>{{}}</{color}> | sim={{:.3f}}s',
        'STARTUP' if previous is None else previous.name, phase.name, simulation_time_s)


def log_waypoint(index, target_position_m, simulation_time_s, *, settled=False):
    color = SETTLED_WAYPOINT_COLOR if settled else ACTIVE_WAYPOINT_COLOR
    logger.opt(colors=True).info(
        f'Position square: <{color}>waypoint {{}} {{}}</{color}> | target={{}} m | sim={{:.3f}}s',
        index, 'settled' if settled else 'active', target_position_m, simulation_time_s)
