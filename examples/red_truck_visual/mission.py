"""Image-error Acro mission using only explicit MAVLink sensor feedback."""
from enum import StrEnum
from math import cos, degrees, radians, sqrt, tan

from bullet_drone.controllers import AltitudeController
from bullet_drone.controllers.internal import AttitudeController
from bullet_drone.geometry import yaw_from_quaternion
from bullet_drone.rc import RcCodec
from examples.red_truck_visual.controllers.altitude import (
    ALTITUDE_POSITION_PID as ALTITUDE_POSITION_PID,
    ALTITUDE_VELOCITY_PID as ALTITUDE_VELOCITY_PID,
)
from examples.red_truck_visual.controllers.attitude import (
    ATTITUDE_KP_S_INV as ATTITUDE_KP_S_INV,
    MAX_BODY_RATES_RAD_S as MAX_BODY_RATES_RAD_S,
)
from examples.red_truck_visual.controllers.image import (
    FULL_ALTITUDE_APPROACH_DESCENT_M_S as FULL_ALTITUDE_APPROACH_DESCENT_M_S,
    FULL_IMAGE_GUIDANCE_ALTITUDE_M as FULL_IMAGE_GUIDANCE_ALTITUDE_M,
    IMAGE_DESCENT_GAIN_M_S as IMAGE_DESCENT_GAIN_M_S,
    IMAGE_D_FILTER_S as IMAGE_D_FILTER_S,
    IMAGE_PITCH_GAIN_RAD as IMAGE_PITCH_GAIN_RAD,
    IMAGE_ROLL_D_GAIN_RAD_S as IMAGE_ROLL_D_GAIN_RAD_S,
    IMAGE_ROLL_GAIN_RAD as IMAGE_ROLL_GAIN_RAD,
    LOW_ALTITUDE_APPROACH_DESCENT_M_S as LOW_ALTITUDE_APPROACH_DESCENT_M_S,
    MAX_FORWARD_PITCH_RAD as MAX_FORWARD_PITCH_RAD,
    MAX_TILT_RAD as MAX_TILT_RAD,
    MIN_IMAGE_GUIDANCE_SCALE as MIN_IMAGE_GUIDANCE_SCALE,
    PITCH_SLEW_RAD_S as PITCH_SLEW_RAD_S,
    ImageGuidanceController,
)
from examples.red_truck_visual.vision import TargetOffset


TARGET_ALTITUDE_AGL_M = 70.0
SEARCH_TIMEOUT_S = 30.0
MAX_MISSION_TIME_S = 150.0
TAKEOFF_SPEED_M_S = 4.0
DESCENT_SPEED_M_S = 8.0
LANDING_SPEED_M_S = 1.5
GROUND_ALTITUDE_M = 0.0
GROUND_SETTLE_S = 0.5
TAKEOFF_SETTLE_S = 0.3
TOUCHDOWN_SETTLE_S = 0.5
DISARMED_SETTLE_S = 1.0
ALTITUDE_TOLERANCE_M = 1.0
TAKEOFF_VERTICAL_SPEED_TOLERANCE_M_S = 1.5
VERTICAL_SPEED_TOLERANCE_M_S = 0.5
TOUCHDOWN_ALTITUDE_M = 0.03
SEARCH_YAW_RATE_RAD_S = 0.35
REACQUIRE_YAW_RATE_RAD_S = 0.25
CENTER_YAW_GAIN_RAD_S_PER_DX = 1.5
CENTER_MAX_YAW_RATE_RAD_S = 0.6
CENTER_DX_TOLERANCE = 0.08
CENTER_SETTLE_S = 0.3
RC_MIN_PWM = 1000
RC_MAX_PWM = 2000
RC_NEUTRAL_PWM = 1500
RC_MIN_CHECK_PWM = 1050
MOTOR_IDLE_FRACTION = 0.055
ACTUAL_CENTER_RATE_DPS = 70.0
ACTUAL_MAX_RATE_DPS = 670.0
ACTUAL_RATE_CURVE_DPS = ACTUAL_MAX_RATE_DPS - ACTUAL_CENTER_RATE_DPS
UNUSED_CHANNELS = (RC_MIN_PWM,) * 12


class Phase(StrEnum):
    PREARM = 'prearm'
    ARMING = 'arming'
    TAKEOFF = 'takeoff'
    SEARCH = 'search'
    CENTER = 'center'
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
                 codec: RcCodec, max_thrust_n: float, *, camera_fov_x_deg=90.0, camera_aspect=4 / 3):
        self.altitude = altitude
        self.attitude = attitude
        self.codec = codec
        self.max_thrust_n = max_thrust_n
        self.camera_tan_x = tan(radians(camera_fov_x_deg) / 2)
        self.camera_tan_y = self.camera_tan_x / camera_aspect
        self.phase = Phase.PREARM
        self.phase_started_s = 0.0
        self.dwell_started_s = None
        self.search_started_s = 0.0
        self.heading_rad = 0.0
        self.image_guidance = ImageGuidanceController()
        self.transitions = []

    def reset(self, now, sensors):
        self.altitude.reset()
        self.phase = Phase.PREARM
        self.phase_started_s = now
        self.dwell_started_s = None
        self.search_started_s = 0.0
        self.heading_rad = yaw_from_quaternion(sensors.attitude.orientation_xyzw)
        self.image_guidance.reset()
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
            self.image_guidance.reset()

    def _settled(self, condition, now, duration):
        if not condition:
            self.dwell_started_s = None
        elif self.dwell_started_s is None:
            self.dwell_started_s = now
        return self.dwell_started_s is not None and now - self.dwell_started_s >= duration

    def command(self, now, status, dt_s: float, target: TargetOffset | None = None,
                *, target_frame_step: int | None = None,
                contact: bool = False, sensors=None) -> tuple[int, ...]:
        if sensors is None:
            raise RuntimeError('Betaflight MAVLink sensor data is required')
        self._altitude_m = sensors.altitude_m
        vertical_speed = sensors.vertical_speed_m_s
        orientation = sensors.attitude.orientation_xyzw
        yaw = yaw_from_quaternion(orientation)

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
                             and abs(vertical_speed) <= TAKEOFF_VERTICAL_SPEED_TOLERANCE_M_S,
                             now, TAKEOFF_SETTLE_S):
                self._transition(Phase.SEARCH, now)
        elif self.phase == Phase.SEARCH:
            if target is not None:
                self.heading_rad = yaw
                self._transition(Phase.CENTER, now)
            elif now - self.search_started_s >= SEARCH_TIMEOUT_S:
                self._transition(Phase.LAND, now)
        elif self.phase == Phase.CENTER:
            if target is None:
                self._transition(Phase.REACQUIRE, now)
            elif self._settled(abs(target.dx) <= CENTER_DX_TOLERANCE,
                               now, CENTER_SETTLE_S):
                self._transition(Phase.APPROACH, now)
        elif self.phase == Phase.APPROACH and target is None:
            self._transition(Phase.REACQUIRE, now)
        elif self.phase == Phase.REACQUIRE and target is not None:
            self.heading_rad = yaw
            self._transition(Phase.CENTER, now)
        elif self.phase == Phase.LAND:
            if self._settled(self._altitude_m <= TOUCHDOWN_ALTITUDE_M
                             and abs(vertical_speed) <= VERTICAL_SPEED_TOLERANCE_M_S,
                             now, TOUCHDOWN_SETTLE_S):
                self._transition(Phase.DISARMED, now)
        elif self.phase == Phase.DISARMED and not status.armed and now - self.phase_started_s >= DISARMED_SETTLE_S:
            self._transition(Phase.COMPLETE, now)

        flight = self.phase in (Phase.TAKEOFF, Phase.SEARCH, Phase.CENTER, Phase.APPROACH,
                                Phase.REACQUIRE, Phase.LAND)
        arm = self.phase in (Phase.ARMING, Phase.TAKEOFF, Phase.SEARCH, Phase.CENTER,
                             Phase.APPROACH, Phase.REACQUIRE, Phase.LAND)
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
                pitch_target = self.image_guidance.level_pitch(dt_s)
        elif self.phase == Phase.CENTER:
            vertical_target = self._altitude_m
            pitch_target = self.image_guidance.level_pitch(dt_s)
            if target is not None:
                yaw_rate_override = (0.0 if abs(target.dx) <= CENTER_DX_TOLERANCE else
                    max(-CENTER_MAX_YAW_RATE_RAD_S,
                        min(CENTER_MAX_YAW_RATE_RAD_S,
                            -target.dx * CENTER_YAW_GAIN_RAD_S_PER_DX)))
        elif self.phase == Phase.APPROACH:
            if target is None:
                target = TargetOffset(0.0, 0.0, 0)
            roll_target, pitch_target, vertical_target = self.image_guidance.approach(
                target, self._altitude_m, dt_s, target_frame_step)
        elif self.phase == Phase.LAND:
            vertical_target = GROUND_ALTITUDE_M

        thrust = self.altitude.thrust(vertical_target, self._altitude_m, vertical_speed, dt_s,
            velocity_limit_m_s=(DESCENT_SPEED_M_S if self.phase == Phase.APPROACH
                                else LANDING_SPEED_M_S if self.phase == Phase.LAND
                                else TAKEOFF_SPEED_M_S if self.phase == Phase.TAKEOFF else 1.0))
        tilt_cosine = max(0.5, cos(roll_target) * cos(pitch_target))
        thrust = min(self.max_thrust_n, thrust / tilt_cosine)
        rates = self.attitude.rates((roll_target, pitch_target, self.heading_rad),
                                    sensors.attitude)
        if yaw_rate_override is not None:
            rates = rates[0], rates[1], yaw_rate_override
        self.control_targets = (pitch_target, vertical_target, thrust)
        return (rate_pwm(rates[0]), rate_pwm(rates[1]),
                throttle_pwm(thrust, self.max_thrust_n), rate_pwm(rates[2]),
                RC_MAX_PWM, RC_MIN_PWM) + UNUSED_CHANNELS
