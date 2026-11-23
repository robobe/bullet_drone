"""State transitions and wall-clock 10 Hz visual-control diagnostics."""
from math import degrees
import time

from loguru import logger

from examples.red_truck_visual.mission import Phase

REPORT_PERIOD_S = 0.1
PHASE_COLORS = {
    Phase.PREARM: 'yellow', Phase.ARMING: 'magenta', Phase.TAKEOFF: 'cyan',
    Phase.SEARCH: 'blue', Phase.APPROACH: 'green', Phase.REACQUIRE: 'yellow',
    Phase.LAND: 'magenta', Phase.DISARMED: 'yellow', Phase.COMPLETE: 'green',
}


class Diagnostics:
    def __init__(self):
        self.transition_count = 0
        self.next_report = 0.0

    def report(self, mission, state, target, channels):
        for index in range(self.transition_count, len(mission.transitions)):
            phase, simulation_time = mission.transitions[index]
            previous = mission.transitions[index - 1][0].name if index else 'STARTUP'
            color = PHASE_COLORS[phase]
            logger.opt(colors=True).info(
                f'STATE {{}} -> <{color}>{{}}</{color}> | sim={{:.3f}}s',
                previous, phase.name, simulation_time)
        self.transition_count = len(mission.transitions)
        now = time.monotonic()
        if now < self.next_report:
            return
        self.next_report = now + REPORT_PERIOD_S
        error = ('LOST' if target is None else
                 f'dx={target.dx:+.4f} dy={target.dy:+.4f} red_pixels={target.pixels}')
        pitch, altitude, thrust = mission.control_targets
        logger.info(
            'VISION phase={} sim={:.2f}s target={} | z={:.2f} z_target={:.2f}m '
            'vz={:+.2f}m/s pitch_target={:+.1f}deg thrust={:.2f}N | '
            'RC roll={} pitch={} throttle={} yaw={}',
            mission.phase.name, state.simulation_time_s, error, state.position_m[2],
            altitude, state.linear_velocity_m_s[2], degrees(pitch), thrust,
            channels[0], channels[1], channels[2], channels[3])
