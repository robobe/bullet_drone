"""Image-servo Acro mission; renderer pixels steer, physics state stabilizes."""
from enum import StrEnum
from math import cos, degrees, radians, sin, sqrt

from bullet_drone.controllers import AltitudeController, IdealStateFeedback, PidSettings
from bullet_drone.controllers.internal import AttitudeController
from bullet_drone.geometry import yaw_from_quaternion
from bullet_drone.rc import RcCodec
from examples.red_truck_visual.vision import TargetOffset


TARGET_ALTITUDE_AGL_M = 30.0
SEARCH_TIMEOUT_S = 30.0
MAX_MISSION_TIME_S = 150.0
APPROACH_SPEED_M_S = 6.0
DESCENT_SPEED_M_S = 3.0
LANDING_SPEED_M_S = 1.5
GROUND_ALTITUDE_M = 0.0
GROUND_SETTLE_S = 0.5
TAKEOFF_SETTLE_S = 1.0
TOUCHDOWN_SETTLE_S = 0.5
DISARMED_SETTLE_S = 1.0
ALTITUDE_TOLERANCE_M = 0.15
VERTICAL_SPEED_TOLERANCE_M_S = 0.15
TOUCHDOWN_ALTITUDE_M = 0.03
SEARCH_YAW_RATE_RAD_S = 0.35
REACQUIRE_YAW_RATE_RAD_S = 0.25
MAX_TILT_RAD = radians(30.0)
MAX_BODY_RATES_RAD_S = (2.0, 2.0, 1.0)
ATTITUDE_KP_S_INV = (4.0, 4.0, 4.0)
IMAGE_ROLL_GAIN_RAD = radians(12.0)
IMAGE_PITCH_GAIN_RAD = radians(20.0)
MAX_FORWARD_PITCH_RAD = radians(25.0)
FORWARD_SPEED_GAIN_RAD_PER_M_S = 0.08
VERTICAL_IMAGE_TRIM_M = 8.0
PITCH_SLEW_RAD_S = radians(45.0)
RC_MIN_PWM = 1000
RC_MAX_PWM = 2000
RC_NEUTRAL_PWM = 1500
RC_MIN_CHECK_PWM = 1050
MOTOR_IDLE_FRACTION = 0.055
ACTUAL_CENTER_RATE_DPS = 70.0
ACTUAL_MAX_RATE_DPS = 670.0
ACTUAL_RATE_CURVE_DPS = ACTUAL_MAX_RATE_DPS - ACTUAL_CENTER_RATE_DPS
UNUSED_CHANNELS = (RC_MIN_PWM,) * 12
ALTITUDE_POSITION_PID = PidSettings(1.5, 0.0, 0.0, 3.0, 0.0)
ALTITUDE_VELOCITY_PID = PidSettings(6.0, 4.0, 0.0, 5.0, 2.0)


class Phase(StrEnum):
    PREARM = 'prearm'
    ARMING = 'arming'
    TAKEOFF = 'takeoff'
    SEARCH = 'search'
    APPROACH = 'approach'
    REACQUIRE = 'reacquire'
    LAND = 'land'
    DISARMED = 'disarmed'
    COMPLETE = 'complete'


def throttle_pwm(thrust_n, max_thrust_n):
    if thrust_n <= 0:
        return RC_MIN_PWM
    speed = sqrt(min(1.0, thrust_n / max_thrust_n))
    stick = max(0.0, min(1.0, (speed - MOTOR_IDLE_FRACTION) / (1.0 - MOTOR_IDLE_FRACTION)))
    return round(RC_MIN_CHECK_PWM + (RC_MAX_PWM - RC_MIN_CHECK_PWM) * stick)


def rate_pwm(rate_rad_s):
    target_dps = max(-ACTUAL_MAX_RATE_DPS, min(ACTUAL_MAX_RATE_DPS, degrees(rate_rad_s)))
    sign = -1.0 if target_dps < 0 else 1.0
    magnitude = abs(target_dps)
    deflection = ((sqrt(ACTUAL_CENTER_RATE_DPS ** 2
                       + 4 * ACTUAL_RATE_CURVE_DPS * magnitude)
                   - ACTUAL_CENTER_RATE_DPS) / (2 * ACTUAL_RATE_CURVE_DPS))
    return round(RC_NEUTRAL_PWM + sign * 500 * deflection)


class RedTruckMission:
    def __init__(self, altitude: AltitudeController, attitude: AttitudeController,
                 codec: RcCodec, max_thrust_n: float):
        self.altitude = altitude
        self.attitude = attitude
        self.codec = codec
        self.max_thrust_n = max_thrust_n
        self.feedback = IdealStateFeedback()
        self.phase = Phase.PREARM
        self.phase_started_s = 0.0
        self.dwell_started_s = None
        self.search_started_s = 0.0
        self.approach_started_s = 0.0
        self.approach_start_altitude_m = TARGET_ALTITUDE_AGL_M
        self.heading_rad = 0.0
        self.last_pitch_rad = 0.0
        self.transitions = []

    def reset(self, observation):
        state = observation.ground_truth
        self.altitude.reset()
        self.phase = Phase.PREARM
        self.phase_started_s = state.simulation_time_s
        self.dwell_started_s = None
        self.search_started_s = 0.0
        self.approach_started_s = 0.0
        self.approach_start_altitude_m = TARGET_ALTITUDE_AGL_M
        self.heading_rad = yaw_from_quaternion(state.orientation_xyzw)
        self.last_pitch_rad = 0.0
        self.transitions = [(self.phase, self.phase_started_s)]

    def _transition(self, phase, now):
        self.phase = phase
        self.phase_started_s = now
        self.dwell_started_s = None
        self.transitions.append((phase, now))
        if phase in (Phase.TAKEOFF, Phase.LAND, Phase.DISARMED):
            self.altitude.reset()
        if phase == Phase.SEARCH:
            self.search_started_s = now
        if phase == Phase.APPROACH:
            self.approach_started_s = now
            self.approach_start_altitude_m = self._altitude_m
            self.last_pitch_rad = 0.0

    def _settled(self, condition, now, duration):
        if not condition:
            self.dwell_started_s = None
        elif self.dwell_started_s is None:
            self.dwell_started_s = now
        return self.dwell_started_s is not None and now - self.dwell_started_s >= duration

    def command(self, observation, status, dt_s: float, target: TargetOffset | None = None,
                *, contact: bool = False) -> tuple[int, ...]:
        state = observation.ground_truth
        now = state.simulation_time_s
        self._altitude_m = state.position_m[2]
        vertical_speed = state.linear_velocity_m_s[2]
        yaw = yaw_from_quaternion(state.orientation_xyzw)

        if contact:
            self._transition(Phase.COMPLETE, now)
        elif (self.phase not in (Phase.LAND, Phase.DISARMED, Phase.COMPLETE)
              and now - self.transitions[0][1] >= MAX_MISSION_TIME_S):
            self._transition(Phase.LAND, now)
        elif self.phase == Phase.PREARM:
            if self._settled(not status.armed and status.arming_disabled_flags == 0,
                             now, GROUND_SETTLE_S):
                self._transition(Phase.ARMING, now)
        elif self.phase == Phase.ARMING:
            if status.armed:
                self._transition(Phase.TAKEOFF, now)
            elif status.arming_disabled_flags:
                self._transition(Phase.PREARM, now)
        elif self.phase == Phase.TAKEOFF:
            if self._settled(abs(self._altitude_m - TARGET_ALTITUDE_AGL_M) <= ALTITUDE_TOLERANCE_M
                             and abs(vertical_speed) <= VERTICAL_SPEED_TOLERANCE_M_S,
                             now, TAKEOFF_SETTLE_S):
                self._transition(Phase.SEARCH, now)
        elif self.phase == Phase.SEARCH:
            if target is not None:
                self.heading_rad = yaw
                self._transition(Phase.APPROACH, now)
            elif now - self.search_started_s >= SEARCH_TIMEOUT_S:
                self._transition(Phase.LAND, now)
        elif self.phase == Phase.APPROACH and target is None:
            self._transition(Phase.REACQUIRE, now)
        elif self.phase == Phase.REACQUIRE and target is not None:
            self.heading_rad = yaw
            self._transition(Phase.APPROACH, now)
        elif self.phase == Phase.LAND:
            if self._settled(self._altitude_m <= TOUCHDOWN_ALTITUDE_M
                             and abs(vertical_speed) <= VERTICAL_SPEED_TOLERANCE_M_S,
                             now, TOUCHDOWN_SETTLE_S):
                self._transition(Phase.DISARMED, now)
        elif self.phase == Phase.DISARMED and not status.armed and now - self.phase_started_s >= DISARMED_SETTLE_S:
            self._transition(Phase.COMPLETE, now)

        flight = self.phase in (Phase.TAKEOFF, Phase.SEARCH, Phase.APPROACH, Phase.REACQUIRE, Phase.LAND)
        arm = self.phase in (Phase.ARMING, Phase.TAKEOFF, Phase.SEARCH, Phase.APPROACH, Phase.REACQUIRE, Phase.LAND)
        if flight and not status.armed:
            raise RuntimeError(
                f'Betaflight disarmed during {self.phase} at {now:.2f}s, '
                f'altitude={self._altitude_m:.2f}m, '
                f'arming_disabled_flags=0x{status.arming_disabled_flags:08x}')
        if not flight:
            self.control_targets = (0.0, self._altitude_m, 0.0)
            return (RC_NEUTRAL_PWM, RC_NEUTRAL_PWM, RC_MIN_PWM, RC_NEUTRAL_PWM,
                    RC_MAX_PWM if arm else RC_MIN_PWM, RC_MIN_PWM) + UNUSED_CHANNELS

        vertical_target = TARGET_ALTITUDE_AGL_M
        roll_target = pitch_target = 0.0
        yaw_rate_override = None
        if self.phase in (Phase.SEARCH, Phase.REACQUIRE):
            yaw_rate_override = SEARCH_YAW_RATE_RAD_S if self.phase == Phase.SEARCH else REACQUIRE_YAW_RATE_RAD_S
            vertical_target = self._altitude_m
            if self.phase == Phase.REACQUIRE:
                forward_speed = (state.linear_velocity_m_s[0] * cos(yaw)
                                 + state.linear_velocity_m_s[1] * sin(yaw))
                pitch_target = max(-MAX_TILT_RAD, min(0.0,
                    -forward_speed * FORWARD_SPEED_GAIN_RAD_PER_M_S))
                max_change = PITCH_SLEW_RAD_S * dt_s
                pitch_target = max(self.last_pitch_rad - max_change,
                                   min(self.last_pitch_rad + max_change, pitch_target))
                self.last_pitch_rad = pitch_target
        elif self.phase == Phase.APPROACH:
            if target is None:
                target = TargetOffset(0.0, 0.0, 0)
            roll_target = max(-MAX_TILT_RAD, min(MAX_TILT_RAD, target.dx * IMAGE_ROLL_GAIN_RAD))
            forward_speed = (state.linear_velocity_m_s[0] * cos(yaw)
                             + state.linear_velocity_m_s[1] * sin(yaw))
            forward_pitch = max(0.0, min(MAX_FORWARD_PITCH_RAD,
                (APPROACH_SPEED_M_S - forward_speed) * FORWARD_SPEED_GAIN_RAD_PER_M_S))
            pitch_target = max(-MAX_TILT_RAD, min(MAX_TILT_RAD,
                forward_pitch + target.dy * IMAGE_PITCH_GAIN_RAD))
            max_change = PITCH_SLEW_RAD_S * dt_s
            pitch_target = max(self.last_pitch_rad - max_change,
                               min(self.last_pitch_rad + max_change, pitch_target))
            self.last_pitch_rad = pitch_target
            vertical_target = max(GROUND_ALTITUDE_M, min(TARGET_ALTITUDE_AGL_M,
                self.approach_start_altitude_m - DESCENT_SPEED_M_S * (now - self.approach_started_s)
                - target.dy * VERTICAL_IMAGE_TRIM_M))
        elif self.phase == Phase.LAND:
            vertical_target = GROUND_ALTITUDE_M

        thrust = self.altitude.thrust(vertical_target, self._altitude_m, vertical_speed, dt_s,
            velocity_limit_m_s=(DESCENT_SPEED_M_S if self.phase == Phase.APPROACH
                                else LANDING_SPEED_M_S if self.phase == Phase.LAND else 1.0))
        tilt_cosine = max(0.5, cos(roll_target) * cos(pitch_target))
        thrust = min(self.max_thrust_n, thrust / tilt_cosine)
        rates = self.attitude.rates((roll_target, pitch_target, self.heading_rad),
                                    self.feedback.read(observation))
        if yaw_rate_override is not None:
            rates = rates[0], rates[1], yaw_rate_override
        self.control_targets = (pitch_target, vertical_target, thrust)
        return (rate_pwm(rates[0]), rate_pwm(rates[1]),
                throttle_pwm(thrust, self.max_thrust_n), rate_pwm(rates[2]),
                RC_MAX_PWM, RC_MIN_PWM) + UNUSED_CHANNELS
