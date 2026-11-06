"""Independent altitude mission; world Z and velocity in SI, RC sent through MSP."""
from enum import StrEnum
from math import sqrt

from bullet_drone.controllers import AltitudeController, PidSettings


TARGET_ALTITUDE_M = 3.0
GROUND_ALTITUDE_M = 0.0
GROUND_SETTLE_S = 0.5
HOLD_DURATION_S = 10.0
HOVER_DWELL_S = 1.0
HOVER_ALTITUDE_TOLERANCE_M = 0.05
HOVER_SPEED_TOLERANCE_M_S = 0.1
TOUCHDOWN_ALTITUDE_M = 0.02
TOUCHDOWN_SPEED_M_S = 0.05
TOUCHDOWN_DWELL_S = 0.5
DISARMED_DURATION_S = 1.0
FLIGHT_SPEED_LIMIT_M_S = 1.0
LANDING_SPEED_LIMIT_M_S = 0.5
RC_MIN_PWM = 1000
RC_MAX_PWM = 2000
RC_NEUTRAL_PWM = 1500
RC_MIN_CHECK_PWM = 1050
MOTOR_IDLE_FRACTION = 0.055
UNUSED_CHANNELS = (RC_MIN_PWM,) * 12
WORLD_Z_AXIS = 2
POSITION_PID = PidSettings(1.5, 0.0, 0.0, FLIGHT_SPEED_LIMIT_M_S, 0.0)
VELOCITY_PID = PidSettings(6.0, 4.0, 0.0, 3.0, 2.0)


class Phase(StrEnum):
    PREARM = 'prearm'
    ARMING = 'arming'
    TAKEOFF = 'takeoff'
    HOLD = 'hold'
    LAND = 'land'
    DISARMED = 'disarmed'
    COMPLETE = 'complete'


def throttle_pwm(thrust_n, max_thrust_n):
    """Invert normal-mode speed mixing: thrust ~ speed², idle=5.5%, min_check=1050."""
    if thrust_n <= 0:
        return RC_MIN_PWM
    speed = sqrt(min(1.0, thrust_n / max_thrust_n))
    stick = max(0.0, min(1.0, (speed - MOTOR_IDLE_FRACTION) / (1.0 - MOTOR_IDLE_FRACTION)))
    return round(RC_MIN_CHECK_PWM + (RC_MAX_PWM - RC_MIN_CHECK_PWM) * stick)


class AltitudeHoldMission:
    def __init__(self, mass_kg, gravity_m_s2, max_thrust_n):
        self.controller = AltitudeController(mass_kg, gravity_m_s2, max_thrust_n, POSITION_PID, VELOCITY_PID)
        self.max_thrust_n = max_thrust_n
        self.phase = Phase.PREARM
        self.phase_started_s = 0.0
        self.dwell_started_s = None
        self.transitions = [(self.phase, self.phase_started_s)]

    def transition(self, phase, now_s):
        self.phase = phase
        self.phase_started_s = now_s
        self.dwell_started_s = None
        self.transitions.append((phase, now_s))
        if phase in (Phase.TAKEOFF, Phase.LAND, Phase.DISARMED):
            self.controller.reset()

    def settled(self, condition, now_s, duration_s):
        if not condition:
            self.dwell_started_s = None
        elif self.dwell_started_s is None:
            self.dwell_started_s = now_s
        return self.dwell_started_s is not None and now_s - self.dwell_started_s >= duration_s

    def command(self, state, status, dt_s):
        now = state.simulation_time_s
        altitude, speed = state.position_m[WORLD_Z_AXIS], state.linear_velocity_m_s[WORLD_Z_AXIS]
        if self.phase == Phase.PREARM:
            if self.settled(not status.armed and status.arming_disabled_flags == 0,
                            now, GROUND_SETTLE_S):
                self.transition(Phase.ARMING, now)
        elif self.phase == Phase.ARMING:
            if status.armed:
                self.transition(Phase.TAKEOFF, now)
            elif status.arming_disabled_flags:
                # Firmware clears NOT_DISARMED/ARM_SWITCH only after receiving arm low.
                self.transition(Phase.PREARM, now)
        elif self.phase == Phase.TAKEOFF:
            if self.settled(abs(altitude - TARGET_ALTITUDE_M) <= HOVER_ALTITUDE_TOLERANCE_M
                            and abs(speed) <= HOVER_SPEED_TOLERANCE_M_S, now, HOVER_DWELL_S):
                self.transition(Phase.HOLD, now)
        elif self.phase == Phase.HOLD:
            if now - self.phase_started_s >= HOLD_DURATION_S:
                self.transition(Phase.LAND, now)
        elif self.phase == Phase.LAND:
            if self.settled(altitude <= TOUCHDOWN_ALTITUDE_M and abs(speed) <= TOUCHDOWN_SPEED_M_S,
                            now, TOUCHDOWN_DWELL_S):
                self.transition(Phase.DISARMED, now)
        elif self.phase == Phase.DISARMED:
            if not status.armed and now - self.phase_started_s >= DISARMED_DURATION_S:
                self.transition(Phase.COMPLETE, now)

        flight = self.phase in (Phase.TAKEOFF, Phase.HOLD, Phase.LAND)
        if flight and not status.armed:
            raise RuntimeError('Betaflight disarmed during flight')
        thrust = 0.0
        if flight:
            landing = self.phase == Phase.LAND
            thrust = self.controller.thrust(GROUND_ALTITUDE_M if landing else TARGET_ALTITUDE_M,
                altitude, speed, dt_s,
                velocity_limit_m_s=LANDING_SPEED_LIMIT_M_S if landing else FLIGHT_SPEED_LIMIT_M_S)
        arm = self.phase in (Phase.ARMING, Phase.TAKEOFF, Phase.HOLD, Phase.LAND)
        return (RC_NEUTRAL_PWM, RC_NEUTRAL_PWM, throttle_pwm(thrust, self.max_thrust_n),
                RC_NEUTRAL_PWM, RC_MAX_PWM if arm else RC_MIN_PWM, RC_MAX_PWM) + UNUSED_CHANNELS
