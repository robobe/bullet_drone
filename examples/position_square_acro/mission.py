"""Independent XYZ square mission producing Betaflight Acro MSP channels."""

from dataclasses import dataclass
from enum import StrEnum
from math import degrees, dist, hypot, radians, sqrt

from bullet_drone.controllers import AltitudeController, IdealStateFeedback, PositionController, VelocityController
from bullet_drone.controllers.internal import AttitudeController
from bullet_drone.geometry import yaw_from_quaternion
from bullet_drone.models import DroneObservation, DroneState


WORLD_Z_AXIS = 2
XY_AXIS_COUNT = 2
TARGET_ALTITUDE_M = 3.0
GROUND_ALTITUDE_M = 0.0
SIDE_LENGTH_M = 3.0
GROUND_SETTLE_S = 0.5
ARRIVAL_POSITION_TOLERANCE_M = 0.1
ARRIVAL_SPEED_TOLERANCE_M_S = 0.1
ARRIVAL_SETTLE_S = 1.0
HOLD_DURATION_S = 10.0
TOUCHDOWN_XY_TOLERANCE_M = 0.1
TOUCHDOWN_ALTITUDE_M = 0.02
TOUCHDOWN_SPEED_M_S = 0.05
TOUCHDOWN_SETTLE_S = 0.5
DISARMED_DURATION_S = 1.0
FLIGHT_SPEED_LIMIT_M_S = 1.0
LANDING_SPEED_LIMIT_M_S = 0.5
RC_MIN_PWM = 1000
RC_MAX_PWM = 2000
RC_NEUTRAL_PWM = 1500
RC_MIN_CHECK_PWM = 1050
MOTOR_IDLE_FRACTION = 0.055
ACTUAL_CENTER_RATE_DPS = 70.0
ACTUAL_MAX_RATE_DPS = 670.0
ACTUAL_RATE_CURVE_DPS = ACTUAL_MAX_RATE_DPS - ACTUAL_CENTER_RATE_DPS
UNUSED_CHANNELS = (RC_MIN_PWM,) * 12
ROUTE_OFFSETS_XY_M = ((0.0, 0.0), (SIDE_LENGTH_M, 0.0),
                      (SIDE_LENGTH_M, SIDE_LENGTH_M), (0.0, SIDE_LENGTH_M), (0.0, 0.0))


class Phase(StrEnum):
    PREARM = 'prearm'
    ARMING = 'arming'
    TAKEOFF = 'takeoff'
    SQUARE = 'square'
    HOLD = 'hold'
    LAND = 'land'
    DISARMED = 'disarmed'
    COMPLETE = 'complete'


@dataclass(frozen=True, slots=True)
class Transition:
    phase: Phase
    simulation_time_s: float


@dataclass(frozen=True, slots=True)
class WaypointArrival:
    index: int
    target_position_m: tuple[float, float, float]
    state: DroneState


def throttle_pwm(thrust_n, max_thrust_n):
    """Invert normal-mode speed mixing: thrust is proportional to speed squared."""
    if thrust_n <= 0:
        return RC_MIN_PWM
    speed = sqrt(min(1.0, thrust_n / max_thrust_n))
    stick = max(0.0, min(1.0, (speed - MOTOR_IDLE_FRACTION) / (1.0 - MOTOR_IDLE_FRACTION)))
    return round(RC_MIN_CHECK_PWM + (RC_MAX_PWM - RC_MIN_CHECK_PWM) * stick)


def rate_pwm(rate_rad_s):
    """Invert the configured ACTUAL rate curve (70 deg/s center, 670 deg/s max)."""
    target_dps = max(-ACTUAL_MAX_RATE_DPS, min(ACTUAL_MAX_RATE_DPS, degrees(rate_rad_s)))
    sign = -1.0 if target_dps < 0 else 1.0
    magnitude = abs(target_dps)
    deflection = ((sqrt(ACTUAL_CENTER_RATE_DPS ** 2
                         + 4 * ACTUAL_RATE_CURVE_DPS * magnitude)
                   - ACTUAL_CENTER_RATE_DPS) / (2 * ACTUAL_RATE_CURVE_DPS))
    return round(RC_NEUTRAL_PWM + sign * 500 * deflection)


class SquareMission:
    def __init__(self, position: PositionController, velocity: VelocityController,
                 altitude: AltitudeController, attitude: AttitudeController,
                 max_thrust_n: float):
        self.position = position
        self.velocity = velocity
        self.altitude = altitude
        self.attitude = attitude
        self.feedback = IdealStateFeedback()
        self.max_thrust_n = max_thrust_n
        self.phase = Phase.PREARM
        self.phase_started_s = 0.0
        self.dwell_started_s = None
        self.index = 0
        self.home_xy_m = ROUTE_OFFSETS_XY_M[0]
        self.heading_rad = 0.0
        self.waypoints = ()
        self.transitions = []
        self.arrivals = []

    def _reset_controllers(self):
        for controller in (self.position, self.velocity, self.altitude):
            controller.reset()

    def reset(self, observation: DroneObservation):
        state = observation.ground_truth
        self._reset_controllers()
        self.phase = Phase.PREARM
        self.phase_started_s = state.simulation_time_s
        self.dwell_started_s = None
        self.index = 0
        self.home_xy_m = state.position_m[:XY_AXIS_COUNT]
        self.heading_rad = yaw_from_quaternion(state.orientation_xyzw)
        self.waypoints = tuple((self.home_xy_m[0] + x, self.home_xy_m[1] + y, TARGET_ALTITUDE_M)
                               for x, y in ROUTE_OFFSETS_XY_M)
        self.transitions = [Transition(self.phase, self.phase_started_s)]
        self.arrivals = []

    def _transition(self, phase, now_s):
        self.phase = phase
        self.phase_started_s = now_s
        self.dwell_started_s = None
        self.transitions.append(Transition(phase, now_s))
        if phase in (Phase.TAKEOFF, Phase.LAND, Phase.DISARMED):
            self._reset_controllers()

    def _settled(self, condition, now_s, duration_s):
        if not condition:
            self.dwell_started_s = None
        elif self.dwell_started_s is None:
            self.dwell_started_s = now_s
        return self.dwell_started_s is not None and now_s - self.dwell_started_s >= duration_s

    def command(self, observation, status, dt_s):
        state = observation.ground_truth
        now = state.simulation_time_s
        if self.phase == Phase.PREARM:
            if self._settled(not status.armed and status.arming_disabled_flags == 0,
                             now, GROUND_SETTLE_S):
                self._transition(Phase.ARMING, now)
        elif self.phase == Phase.ARMING:
            if status.armed:
                self._transition(Phase.TAKEOFF, now)
            elif status.arming_disabled_flags:
                self._transition(Phase.PREARM, now)
        elif self.phase in (Phase.TAKEOFF, Phase.SQUARE):
            target = self.waypoints[self.index]
            arrived = (dist(state.position_m, target) <= ARRIVAL_POSITION_TOLERANCE_M
                       and hypot(*state.linear_velocity_m_s) <= ARRIVAL_SPEED_TOLERANCE_M_S)
            if self._settled(arrived, now, ARRIVAL_SETTLE_S):
                self.arrivals.append(WaypointArrival(self.index, target, state))
                if self.index == len(self.waypoints) - 1:
                    self._transition(Phase.HOLD, now)
                else:
                    self.index += 1
                    self.dwell_started_s = None
                    if self.phase == Phase.TAKEOFF:
                        self._transition(Phase.SQUARE, now)
        elif self.phase == Phase.HOLD and now - self.phase_started_s >= HOLD_DURATION_S:
            self._transition(Phase.LAND, now)
        elif self.phase == Phase.LAND:
            touchdown = (dist(state.position_m[:XY_AXIS_COUNT], self.home_xy_m)
                         <= TOUCHDOWN_XY_TOLERANCE_M
                         and state.position_m[WORLD_Z_AXIS] <= TOUCHDOWN_ALTITUDE_M
                         and abs(state.linear_velocity_m_s[WORLD_Z_AXIS]) <= TOUCHDOWN_SPEED_M_S)
            if self._settled(touchdown, now, TOUCHDOWN_SETTLE_S):
                self._transition(Phase.DISARMED, now)
        elif self.phase == Phase.DISARMED:
            if not status.armed and now - self.phase_started_s >= DISARMED_DURATION_S:
                self._transition(Phase.COMPLETE, now)

        flight = self.phase in (Phase.TAKEOFF, Phase.SQUARE, Phase.HOLD, Phase.LAND)
        if flight and not status.armed:
            raise RuntimeError('Betaflight disarmed during flight')
        arm = self.phase in (Phase.ARMING, Phase.TAKEOFF, Phase.SQUARE, Phase.HOLD, Phase.LAND)
        if not flight:
            return (RC_NEUTRAL_PWM, RC_NEUTRAL_PWM, RC_MIN_PWM, RC_NEUTRAL_PWM,
                    RC_MAX_PWM if arm else RC_MIN_PWM, RC_MIN_PWM) + UNUSED_CHANNELS

        landing = self.phase == Phase.LAND
        target = (*self.home_xy_m, GROUND_ALTITUDE_M) if landing else self.waypoints[self.index]
        measured_yaw = yaw_from_quaternion(state.orientation_xyzw)
        target_velocity = self.position.velocity(target[:XY_AXIS_COUNT], state.position_m[:XY_AXIS_COUNT], dt_s)
        vertical_thrust = self.altitude.thrust(
            target[WORLD_Z_AXIS], state.position_m[WORLD_Z_AXIS],
            state.linear_velocity_m_s[WORLD_Z_AXIS], dt_s,
            velocity_limit_m_s=LANDING_SPEED_LIMIT_M_S if landing else FLIGHT_SPEED_LIMIT_M_S)
        output = self.velocity.update(target_velocity, state.linear_velocity_m_s[:XY_AXIS_COUNT],
                                      vertical_thrust, measured_yaw, dt_s)
        rates = self.attitude.rates((output.roll_rad, output.pitch_rad, self.heading_rad),
                                    self.feedback.read(observation))
        return (rate_pwm(rates[0]), rate_pwm(rates[1]),
                throttle_pwm(output.thrust_n, self.max_thrust_n), rate_pwm(rates[2]),
                RC_MAX_PWM, RC_MIN_PWM) + UNUSED_CHANNELS
