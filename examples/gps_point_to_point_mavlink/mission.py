"""Mission consumes only MAVLink feedback, mapped elevations and MSP arm status."""
from math import dist, hypot, radians

from bullet_drone.controllers import AltitudeController, PidSettings, PositionController, VelocityController
from bullet_drone.controllers.internal import AttitudeController
from loguru import logger
from .navigation import line_velocity
from .rc import rate_pwm, throttle_pwm
from .sitl import SAFE_CHANNELS

ARRIVAL_ERROR_M = 0.5
ARRIVAL_SPEED_M_S = 0.2
SETTLE_S = 1.0
TOUCHDOWN_HEIGHT_TOLERANCE_M = 0.12
TOUCHDOWN_SPEED_M_S = 0.1
MAX_TILT_RAD = radians(15)
POSITION_PID = PidSettings(0.8, 0.0, 0.0, 3.0, 0.0)
VELOCITY_PID = PidSettings(2.0, 0.0, 0.0, 1.5, 0.0)
ALTITUDE_POSITION_PID = PidSettings(1.5, 0.0, 0.0, 1.0, 0.0)
ALTITUDE_VELOCITY_PID = PidSettings(6.0, 4.0, 0.0, 3.0, 2.0)
ATTITUDE_GAIN_S_INV = (4.0, 4.0, 4.0)
BODY_RATE_LIMIT_RAD_S = (2.0, 2.0, 2.0)
LANDING_SPEED_M_S = 0.3
CLIMB_SPEED_M_S = 1.0
CRUISE_TARGET_ACCELERATION_M_S2 = 1.0
ARM_PWM = 2000
LOW_PWM = 1000
UNUSED_CHANNELS = (LOW_PWM,) * 12
FLIGHT_PHASES = ('takeoff', 'cruise', 'land')


class Mission:
    def __init__(self, start, end, cruise_z, landing_z, mass, gravity, max_thrust):
        self.start, self.end = start, end
        self.cruise_z, self.landing_z = cruise_z, landing_z
        self.max_thrust = max_thrust
        self.position = PositionController(POSITION_PID)
        self.velocity = VelocityController(mass, max_thrust, VELOCITY_PID, MAX_TILT_RAD)
        self.altitude = AltitudeController(mass, gravity, max_thrust, ALTITUDE_POSITION_PID, ALTITUDE_VELOCITY_PID)
        self.attitude = AttitudeController(ATTITUDE_GAIN_S_INV, BODY_RATE_LIMIT_RAD_S)
        self.phase = 'prearm'
        self.dwell = None
        self.transitions = []
        self.target_velocity = (0.0, 0.0)

    def transition(self, phase, now):
        logger.info('Mission {} -> {} at {:.1f} s', self.phase, phase, now)
        self.phase, self.dwell = phase, None
        self.target_velocity = (0.0, 0.0)
        self.transitions.append((phase, now))
        for controller in (self.position, self.velocity, self.altitude):
            controller.reset()

    def settled(self, condition, now):
        if not condition:
            self.dwell = None
        elif self.dwell is None:
            self.dwell = now
        return self.dwell is not None and now - self.dwell >= SETTLE_S

    def cruise_velocity(self, position, dt):
        desired = line_velocity(self.start, self.end, position)
        delta = tuple(wanted - previous for wanted, previous in zip(desired, self.target_velocity))
        magnitude = hypot(*delta)
        maximum_delta = CRUISE_TARGET_ACCELERATION_M_S2 * dt
        fraction = min(1.0, maximum_delta / magnitude) if magnitude else 1.0
        self.target_velocity = tuple(previous + fraction * change
                                     for previous, change in zip(self.target_velocity, delta))
        return self.target_velocity

    def command(self, feedback, status, now, dt):
        pos, vel = feedback.position_m, feedback.velocity_m_s
        if self.phase == 'prearm':
            if self.settled(not status.armed and status.arming_disabled_flags == 0, now):
                self.transition('arming', now)
        elif self.phase == 'arming':
            if status.armed:
                self.transition('takeoff', now)
            elif status.arming_disabled_flags:
                self.transition('prearm', now)
        elif self.phase == 'takeoff':
            if self.settled(dist(pos, (*self.start, self.cruise_z)) < ARRIVAL_ERROR_M
                            and hypot(*vel) < ARRIVAL_SPEED_M_S, now):
                self.transition('cruise', now)
        elif self.phase == 'cruise':
            if self.settled(dist(pos, (*self.end, self.cruise_z)) < ARRIVAL_ERROR_M
                            and hypot(*vel) < ARRIVAL_SPEED_M_S, now):
                self.transition('land', now)
        elif self.phase == 'land':
            if dist(pos[:2], self.end) > ARRIVAL_ERROR_M:
                self.transition('cruise', now)
            elif self.settled(abs(pos[2] - self.landing_z) < TOUCHDOWN_HEIGHT_TOLERANCE_M
                              and hypot(*vel) < TOUCHDOWN_SPEED_M_S, now):
                self.transition('disarmed', now)
        elif self.phase == 'disarmed':
            if self.settled(not status.armed, now):
                self.transition('complete', now)
        if self.phase in FLIGHT_PHASES and not status.armed:
            raise RuntimeError(f'Firmware disarmed during {self.phase}: modes={status.mode_flags}, blockers={status.arming_disabled_flags}, position={pos}, velocity={vel}')
        if self.phase not in FLIGHT_PHASES:
            return (*SAFE_CHANNELS[:4], ARM_PWM if self.phase == 'arming' else LOW_PWM,
                    LOW_PWM) + UNUSED_CHANNELS
        if self.phase == 'cruise':
            target_velocity = self.cruise_velocity(pos, dt)
        else:
            target = self.start if self.phase == 'takeoff' else self.end
            target_velocity = self.position.velocity(target, pos[:2], dt)
        landing = self.phase == 'land'
        vertical = self.altitude.thrust(self.landing_z if landing else self.cruise_z,
                                       pos[2], vel[2], dt,
                                       LANDING_SPEED_M_S if landing else CLIMB_SPEED_M_S)
        output = self.velocity.update(target_velocity, vel[:2], vertical, feedback.yaw_rad, dt)
        rates = self.attitude.rates((output.roll_rad, output.pitch_rad, 0.0), feedback.attitude)
        return (rate_pwm(rates[0]), rate_pwm(rates[1]), throttle_pwm(output.thrust_n, self.max_thrust),
                rate_pwm(rates[2]), ARM_PWM, LOW_PWM) + UNUSED_CHANNELS
