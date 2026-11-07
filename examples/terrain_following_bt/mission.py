"""Independent terrain-clearance mission producing Betaflight Acro channels."""

from dataclasses import dataclass
from enum import StrEnum
from bisect import bisect_right
from math import atan2, cos, degrees, hypot, pi, radians, sin, sqrt

from bullet_drone.commands import InvalidCommandError
from bullet_drone.controllers import AltitudeController, IdealStateFeedback, PositionController, VelocityController
from bullet_drone.controllers.internal import AttitudeController
from bullet_drone.geometry import yaw_from_quaternion
from bullet_drone.models import DroneObservation, DroneState


XY_AXIS_COUNT = 2
TARGET_CLEARANCE_M = 3.0
LANDING_CLEARANCE_M = 0.0
HEADING_CHANGE_RAD = radians(90.0)
HEADING_TOLERANCE_RAD = radians(3.0)
HEADING_RATE_TOLERANCE_RAD_S = radians(5.0)
GROUND_SETTLE_S = 0.5
TAKEOFF_SETTLE_S = 1.0
WAYPOINT_POSITION_TOLERANCE_M = 0.25
WAYPOINT_SPEED_TOLERANCE_M_S = 0.15
WAYPOINT_SETTLE_S = 0.75
PATH_LOOKAHEAD_M = 4.0
PATH_SEARCH_AHEAD_M = 8.0
TURN_RADIUS_M = 4.0
RETURN_LANE_OFFSET_M = 2.0 * TURN_RADIUS_M
TURN_SAMPLE_COUNT = 64
MERGE_SAMPLE_COUNT = 32
RETURN_MERGE_CONTROL_Y_M = (10.0, 5.0)
TOUCHDOWN_SPEED_TOLERANCE_M_S = 0.15
TOUCHDOWN_SETTLE_S = 0.5
DISARMED_DURATION_S = 1.0
LANDING_SPEED_LIMIT_M_S = 0.5
CLEARANCE_RATE_LIMIT_M_S = 1.5
RC_MIN_PWM = 1000
RC_MAX_PWM = 2000
RC_NEUTRAL_PWM = 1500
RC_MIN_CHECK_PWM = 1050
MOTOR_IDLE_FRACTION = 0.055
ACTUAL_CENTER_RATE_DPS = 70.0
ACTUAL_MAX_RATE_DPS = 670.0
ACTUAL_RATE_CURVE_DPS = ACTUAL_MAX_RATE_DPS - ACTUAL_CENTER_RATE_DPS
UNUSED_CHANNELS = (RC_MIN_PWM,) * 12
OUTBOUND_Y_M = (15.0, 30.0, 45.0, 60.0, 65.0)
RETURN_Y_M = (60.0, 45.0, 30.0, 15.0, 0.0)


class Phase(StrEnum):
    PREARM = 'prearm'
    ARMING = 'arming'
    TAKEOFF = 'takeoff'
    OUTBOUND = 'outbound'
    RETURN = 'return'
    LAND = 'land'
    DISARMED = 'disarmed'
    COMPLETE = 'complete'


@dataclass(frozen=True, slots=True)
class Transition:
    phase: Phase
    simulation_time_s: float


@dataclass(frozen=True, slots=True)
class WaypointArrival:
    phase: Phase
    index: int
    target_xy_m: tuple[float, float]
    state: DroneState
    clearance_m: float


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


class TerrainMission:
    def __init__(self, position: PositionController, velocity: VelocityController,
                 clearance: AltitudeController, attitude: AttitudeController,
                 max_thrust_n: float):
        self.position = position
        self.velocity = velocity
        self.clearance = clearance
        self.attitude = attitude
        self.feedback = IdealStateFeedback()
        self.max_thrust_n = max_thrust_n
        self.phase = Phase.PREARM
        self.phase_started_s = 0.0
        self.dwell_started_s = None
        self.index = 0
        self.home_xy_m = (0.0, 0.0)
        self.heading_rad = 0.0
        self.last_clearance_m = None
        self.last_sensor_time_s = None
        self.clearance_rate_m_s = 0.0
        self.transitions = []
        self.arrivals = []
        self.outbound = ()
        self.return_route = ()
        self.path_xy_m = ()
        self.path_distance_m = ()
        self.outbound_end_distance_m = 0.0
        self.waypoint_distances = ()
        self.next_waypoint = 0
        self.path_progress_m = 0.0

    def _reset_controllers(self):
        self.position.reset()
        self.velocity.reset()
        self.clearance.reset()

    def reset(self, observation: DroneObservation):
        state = observation.ground_truth
        sensor = self._rangefinder(observation)
        self._reset_controllers()
        self.phase = Phase.PREARM
        self.phase_started_s = state.simulation_time_s
        self.dwell_started_s = None
        self.index = 0
        self.home_xy_m = state.position_m[:XY_AXIS_COUNT]
        self.heading_rad = yaw_from_quaternion(state.orientation_xyzw) + HEADING_CHANGE_RAD
        self.outbound = tuple((self.home_xy_m[0], self.home_xy_m[1] + y) for y in OUTBOUND_Y_M)
        lane_x = self.home_xy_m[0] + RETURN_LANE_OFFSET_M
        self.return_route = tuple((lane_x, self.home_xy_m[1] + y) for y in RETURN_Y_M[:-1]) + (self.home_xy_m,)
        path = [self.home_xy_m, *self.outbound]
        self.outbound_end_distance_m = self._append_distance(path)

        center_x = self.home_xy_m[0] + TURN_RADIUS_M
        crest_y = self.home_xy_m[1] + OUTBOUND_Y_M[-1]
        for sample in range(1, TURN_SAMPLE_COUNT + 1):
            angle = pi - pi * sample / TURN_SAMPLE_COUNT
            path.append((center_x + TURN_RADIUS_M * cos(angle),
                         crest_y + TURN_RADIUS_M * sin(angle)))
        path.extend((lane_x, self.home_xy_m[1] + y) for y in RETURN_Y_M[:-1])

        start_x, start_y = self.return_route[-2]
        end_x, end_y = self.home_xy_m
        control_y_1, control_y_2 = RETURN_MERGE_CONTROL_Y_M
        for sample in range(1, MERGE_SAMPLE_COUNT + 1):
            t = sample / MERGE_SAMPLE_COUNT
            reverse_t = 1.0 - t
            x = (reverse_t**3 * start_x + 3 * reverse_t**2 * t * start_x
                 + 3 * reverse_t * t**2 * end_x + t**3 * end_x)
            y = (reverse_t**3 * start_y
                 + 3 * reverse_t**2 * t * (self.home_xy_m[1] + control_y_1)
                 + 3 * reverse_t * t**2 * (self.home_xy_m[1] + control_y_2)
                 + t**3 * end_y)
            path.append((x, y))
        self.path_xy_m = tuple(path)
        distances = [0.0]
        for first, second in zip(path, path[1:]):
            distances.append(distances[-1] + hypot(second[0] - first[0], second[1] - first[1]))
        self.path_distance_m = tuple(distances)
        self.waypoint_distances = tuple(
            (phase, index, point, self._point_distance(point, distances, path, last=phase == Phase.RETURN and index == len(self.return_route) - 1))
            for phase, route in ((Phase.OUTBOUND, self.outbound), (Phase.RETURN, self.return_route))
            for index, point in enumerate(route)
        )
        self.next_waypoint = 0
        self.path_progress_m = 0.0
        self.last_clearance_m = sensor.distance_cm / 100.0
        self.last_sensor_time_s = sensor.simulation_time_s
        self.clearance_rate_m_s = 0.0
        self.transitions = [Transition(self.phase, self.phase_started_s)]
        self.arrivals = []

    @staticmethod
    def _rangefinder(observation):
        sensor = observation.sensors.rangefinder
        state = observation.ground_truth
        if (sensor is None or not sensor.valid or sensor.distance_cm is None
                or sensor.episode_id != state.episode_id
                or sensor.step_index != state.step_index
                or sensor.simulation_time_s != state.simulation_time_s):
            raise InvalidCommandError('Terrain following requires a valid rangefinder reading for the current step')
        return sensor

    def _read_clearance(self, observation):
        """Use AGL rate so terrain changes enter the vertical feedback loop."""
        sensor = self._rangefinder(observation)
        clearance_m = sensor.distance_cm / 100.0
        if self.last_sensor_time_s != sensor.simulation_time_s:
            dt_s = sensor.simulation_time_s - self.last_sensor_time_s
            if dt_s <= 0:
                raise InvalidCommandError('Rangefinder timestamps must increase during flight')
            self.clearance_rate_m_s = (clearance_m - self.last_clearance_m) / dt_s
            self.last_sensor_time_s = sensor.simulation_time_s
            self.last_clearance_m = clearance_m
        return clearance_m

    def _transition(self, phase, now_s):
        self.phase = phase
        self.phase_started_s = now_s
        self.dwell_started_s = None
        self.transitions.append(Transition(phase, now_s))
        if phase in (Phase.TAKEOFF, Phase.OUTBOUND, Phase.LAND, Phase.DISARMED):
            self._reset_controllers()

    def _settled(self, condition, now_s, duration_s):
        if not condition:
            self.dwell_started_s = None
        elif self.dwell_started_s is None:
            self.dwell_started_s = now_s
        return self.dwell_started_s is not None and now_s - self.dwell_started_s >= duration_s

    @staticmethod
    def _append_distance(path):
        return sum(hypot(second[0] - first[0], second[1] - first[1])
                   for first, second in zip(path, path[1:]))

    @staticmethod
    def _point_distance(point, distances, path, *, last=False):
        indices = [index for index, candidate in enumerate(path) if candidate == point]
        return distances[indices[-1] if last else indices[0]]

    def _advance_path(self, position_xy_m):
        """Track forward progress on nearby segments without dwelling at markers."""
        current_segment = max(0, bisect_right(self.path_distance_m, self.path_progress_m) - 1)
        final_segment = min(len(self.path_xy_m) - 2,
                           bisect_right(self.path_distance_m,
                                        self.path_progress_m + PATH_SEARCH_AHEAD_M))
        best_error_sq = float('inf')
        best_progress = self.path_progress_m
        x, y = position_xy_m
        for segment in range(max(0, current_segment - 1), final_segment + 1):
            start, end = self.path_xy_m[segment:segment + 2]
            dx, dy = end[0] - start[0], end[1] - start[1]
            length_sq = dx * dx + dy * dy
            if length_sq == 0:
                continue
            fraction = max(0.0, min(1.0, ((x - start[0]) * dx + (y - start[1]) * dy) / length_sq))
            error_sq = (x - start[0] - fraction * dx) ** 2 + (y - start[1] - fraction * dy) ** 2
            progress = self.path_distance_m[segment] + fraction * sqrt(length_sq)
            if progress >= self.path_progress_m and error_sq < best_error_sq:
                best_error_sq = error_sq
                best_progress = progress
        self.path_progress_m = max(self.path_progress_m, best_progress)
        return self.path_progress_m

    def _lookahead_target(self):
        distance = min(self.path_progress_m + PATH_LOOKAHEAD_M, self.path_distance_m[-1])
        segment = min(bisect_right(self.path_distance_m, distance) - 1,
                      len(self.path_xy_m) - 2)
        start_distance, end_distance = self.path_distance_m[segment:segment + 2]
        fraction = (distance - start_distance) / (end_distance - start_distance)
        start, end = self.path_xy_m[segment:segment + 2]
        return (start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]))

    def _record_passed_waypoints(self, state, clearance_m):
        while (self.next_waypoint < len(self.waypoint_distances)
               and self.path_progress_m >= self.waypoint_distances[self.next_waypoint][3]):
            phase, index, point, _ = self.waypoint_distances[self.next_waypoint]
            self.arrivals.append(WaypointArrival(phase, index, point, state, clearance_m))
            if phase == self.phase:
                self.index = index + 1
            self.next_waypoint += 1

    def command(self, observation, status, dt_s, terrain_contact):
        state = observation.ground_truth
        clearance_m = self._read_clearance(observation)
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
        elif self.phase == Phase.TAKEOFF:
            heading_error = self.heading_rad - yaw_from_quaternion(state.orientation_xyzw)
            yaw_error = atan2(sin(heading_error), cos(heading_error))
            heading_settled = (abs(yaw_error) <= HEADING_TOLERANCE_RAD
                               and abs(state.angular_velocity_rad_s[2]) <= HEADING_RATE_TOLERANCE_RAD_S)
            settled = (heading_settled
                       and abs(clearance_m - TARGET_CLEARANCE_M) <= WAYPOINT_POSITION_TOLERANCE_M
                       and abs(self.clearance_rate_m_s) <= WAYPOINT_SPEED_TOLERANCE_M_S)
            if self._settled(settled, now, TAKEOFF_SETTLE_S):
                self._transition(Phase.OUTBOUND, now)
                self.index = 0
        elif self.phase in (Phase.OUTBOUND, Phase.RETURN):
            progress = self._advance_path(state.position_m[:XY_AXIS_COUNT])
            self._record_passed_waypoints(state, clearance_m)
            if self.phase == Phase.OUTBOUND and progress >= self.outbound_end_distance_m:
                self._transition(Phase.RETURN, now)
                self.index = 0
            elif self.phase == Phase.RETURN:
                home_arrived = (hypot(state.position_m[0] - self.home_xy_m[0],
                                      state.position_m[1] - self.home_xy_m[1])
                                <= WAYPOINT_POSITION_TOLERANCE_M
                                and hypot(*state.linear_velocity_m_s[:XY_AXIS_COUNT])
                                <= WAYPOINT_SPEED_TOLERANCE_M_S)
                if self._settled(home_arrived, now, WAYPOINT_SETTLE_S):
                    self._transition(Phase.LAND, now)
        elif self.phase == Phase.LAND:
            touchdown = (terrain_contact
                         and hypot(*state.linear_velocity_m_s) <= TOUCHDOWN_SPEED_TOLERANCE_M_S)
            if self._settled(touchdown, now, TOUCHDOWN_SETTLE_S):
                self._transition(Phase.DISARMED, now)
        elif self.phase == Phase.DISARMED:
            if not status.armed and now - self.phase_started_s >= DISARMED_DURATION_S:
                self._transition(Phase.COMPLETE, now)

        flight = self.phase in (Phase.TAKEOFF, Phase.OUTBOUND, Phase.RETURN, Phase.LAND)
        if flight and not status.armed:
            raise RuntimeError('Betaflight disarmed during terrain flight')
        arm = self.phase in (Phase.ARMING, Phase.TAKEOFF, Phase.OUTBOUND,
                             Phase.RETURN, Phase.LAND)
        if not flight:
            return (RC_NEUTRAL_PWM, RC_NEUTRAL_PWM, RC_MIN_PWM, RC_NEUTRAL_PWM,
                    RC_MAX_PWM if arm else RC_MIN_PWM, RC_MIN_PWM) + UNUSED_CHANNELS

        if self.phase == Phase.TAKEOFF:
            target_xy = self.home_xy_m
            target_clearance = TARGET_CLEARANCE_M
        elif self.phase == Phase.LAND:
            target_xy = self.home_xy_m
            target_clearance = LANDING_CLEARANCE_M
        else:
            target_xy = self._lookahead_target()
            target_clearance = TARGET_CLEARANCE_M

        measured_yaw = yaw_from_quaternion(state.orientation_xyzw)
        target_velocity = self.position.velocity(target_xy, state.position_m[:XY_AXIS_COUNT], dt_s)
        vertical_thrust = self.clearance.thrust(
            target_clearance, clearance_m, self.clearance_rate_m_s, dt_s,
            velocity_limit_m_s=(LANDING_SPEED_LIMIT_M_S if self.phase == Phase.LAND
                                else CLEARANCE_RATE_LIMIT_M_S))
        output = self.velocity.update(target_velocity, state.linear_velocity_m_s[:XY_AXIS_COUNT],
                                      vertical_thrust, measured_yaw, dt_s)
        rates = self.attitude.rates((output.roll_rad, output.pitch_rad, self.heading_rad),
                                    self.feedback.read(observation))
        return (rate_pwm(rates[0]), rate_pwm(rates[1]),
                throttle_pwm(output.thrust_n, self.max_thrust_n), rate_pwm(rates[2]),
                RC_MAX_PWM, RC_MIN_PWM) + UNUSED_CHANNELS
