"""Own-loop square mission composing XY position, XY velocity, Z, and RC control."""

from dataclasses import dataclass
from enum import StrEnum
from math import dist, hypot

from bullet_drone.commands import RcInput
from bullet_drone.config import ConfigurationError
from bullet_drone.controllers import AltitudeController, IdealStateFeedback, PositionController, VelocityController
from bullet_drone.controllers.internal import AttitudeController
from bullet_drone.geometry import yaw_from_quaternion
from bullet_drone.models import DroneObservation, DroneState
from bullet_drone.rc import RcCodec, heading_rate


WORLD_Z_AXIS = 2
XY_AXIS_COUNT = 2
SIDE_LENGTH_M = 3.0
TARGET_ALTITUDE_M = 3.0
LANDING_ALTITUDE_M = 0.0
GROUND_SETTLE_DURATION_S = 0.5
ARRIVAL_POSITION_TOLERANCE_M = 0.1
ARRIVAL_SPEED_TOLERANCE_M_S = 0.1
ARRIVAL_SETTLE_DURATION_S = 1.0
HOLD_DURATION_S = 10.0
CLIMB_SPEED_LIMIT_M_S = 1.0
DESCENT_SPEED_LIMIT_M_S = 0.5
TOUCHDOWN_XY_TOLERANCE_M = 0.1
TOUCHDOWN_ALTITUDE_M = 0.02
TOUCHDOWN_SPEED_TOLERANCE_M_S = 0.05
TOUCHDOWN_SETTLE_DURATION_S = 0.5
DISARMED_SETTLE_DURATION_S = 1.0
ROUTE_OFFSETS_XY_M = ((0.0, 0.0), (SIDE_LENGTH_M, 0.0),
                      (SIDE_LENGTH_M, SIDE_LENGTH_M), (0.0, SIDE_LENGTH_M), (0.0, 0.0))


class Phase(StrEnum):
    SETTLE = 'settle'
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


class SquareMission:
    def __init__(self, position: PositionController, velocity: VelocityController,
                 altitude: AltitudeController, codec: RcCodec, *, flight_mode: str = 'angle',
                 attitude_controller: AttitudeController | None = None) -> None:
        if flight_mode not in ('angle', 'acro'):
            raise ConfigurationError('Square flight_mode must be angle or acro')
        if flight_mode == 'acro' and attitude_controller is None:
            raise ConfigurationError('Acro position control requires an upstream attitude controller')
        self.flight_mode = flight_mode
        self.attitude_controller = attitude_controller
        self._feedback = IdealStateFeedback()
        self.position = position
        self.velocity = velocity
        self.altitude = altitude
        self.codec = codec
        self.phase = Phase.SETTLE
        self.transitions: list[Transition] = []
        self.arrivals: list[WaypointArrival] = []
        self.index = 0
        self.home_xy_m = ROUTE_OFFSETS_XY_M[0]
        self.heading_rad = 0.0
        self.waypoints: tuple[tuple[float, float, float], ...] = ()
        self._phase_started_s = 0.0
        self._dwell_started_s: float | None = None

    def _reset_controllers(self) -> None:
        for controller in (self.position, self.velocity, self.altitude):
            controller.reset()

    def reset(self, observation: DroneObservation) -> None:
        state = observation.ground_truth
        self._reset_controllers()
        self.phase = Phase.SETTLE
        self.index = 0
        self.home_xy_m = state.position_m[:XY_AXIS_COUNT]
        self.heading_rad = yaw_from_quaternion(state.orientation_xyzw)
        self.waypoints = tuple((self.home_xy_m[0] + x, self.home_xy_m[1] + y, TARGET_ALTITUDE_M)
                               for x, y in ROUTE_OFFSETS_XY_M)
        self._phase_started_s = state.simulation_time_s
        self._dwell_started_s = None
        self.transitions = [Transition(self.phase, self._phase_started_s)]
        self.arrivals = []

    def _transition(self, phase: Phase, now: float) -> None:
        self.phase = phase
        self._phase_started_s = now
        self._dwell_started_s = None
        self.transitions.append(Transition(phase, now))
        if phase in (Phase.TAKEOFF, Phase.LAND, Phase.DISARMED):
            self._reset_controllers()

    def _settled(self, condition: bool, now: float, duration_s: float) -> bool:
        if not condition:
            self._dwell_started_s = None
        elif self._dwell_started_s is None:
            self._dwell_started_s = now
        return self._dwell_started_s is not None and now - self._dwell_started_s >= duration_s

    def command(self, observation: DroneObservation, dt_s: float) -> RcInput:
        state = observation.ground_truth
        now = state.simulation_time_s
        if self.phase == Phase.SETTLE and now - self._phase_started_s >= GROUND_SETTLE_DURATION_S:
            self._transition(Phase.TAKEOFF, now)
        elif self.phase in (Phase.TAKEOFF, Phase.SQUARE):
            target = self.waypoints[self.index]
            arrived = (dist(state.position_m, target) <= ARRIVAL_POSITION_TOLERANCE_M
                       and hypot(*state.linear_velocity_m_s) <= ARRIVAL_SPEED_TOLERANCE_M_S)
            if self._settled(arrived, now, ARRIVAL_SETTLE_DURATION_S):
                self.arrivals.append(WaypointArrival(self.index, target, state))
                if self.index == len(self.waypoints) - 1:
                    self._transition(Phase.HOLD, now)
                else:
                    self.index += 1
                    self._dwell_started_s = None
                    if self.phase == Phase.TAKEOFF:
                        self._transition(Phase.SQUARE, now)
        elif self.phase == Phase.HOLD and now - self._phase_started_s >= HOLD_DURATION_S:
            self._transition(Phase.LAND, now)
        elif self.phase == Phase.LAND:
            touchdown = (dist(state.position_m[:XY_AXIS_COUNT], self.home_xy_m) <= TOUCHDOWN_XY_TOLERANCE_M
                         and state.position_m[WORLD_Z_AXIS] <= TOUCHDOWN_ALTITUDE_M
                         and abs(state.linear_velocity_m_s[WORLD_Z_AXIS]) <= TOUCHDOWN_SPEED_TOLERANCE_M_S)
            if self._settled(touchdown, now, TOUCHDOWN_SETTLE_DURATION_S):
                self._transition(Phase.DISARMED, now)
        elif self.phase == Phase.DISARMED and now - self._phase_started_s >= DISARMED_SETTLE_DURATION_S:
            self._transition(Phase.COMPLETE, now)

        armed = self.phase in (Phase.TAKEOFF, Phase.SQUARE, Phase.HOLD, Phase.LAND)
        if not armed:
            return self.codec.encode(mode=self.flight_mode, armed=False)
        landing = self.phase == Phase.LAND
        target = (*self.home_xy_m, LANDING_ALTITUDE_M) if landing else self.waypoints[self.index]
        measured_yaw = yaw_from_quaternion(state.orientation_xyzw)
        desired_velocity = self.position.velocity(target[:XY_AXIS_COUNT], state.position_m[:XY_AXIS_COUNT], dt_s)
        vertical_thrust = self.altitude.thrust(
            target[WORLD_Z_AXIS], state.position_m[WORLD_Z_AXIS], state.linear_velocity_m_s[WORLD_Z_AXIS], dt_s,
            velocity_limit_m_s=DESCENT_SPEED_LIMIT_M_S if landing else CLIMB_SPEED_LIMIT_M_S,
        )
        output = self.velocity.update(desired_velocity, state.linear_velocity_m_s[:XY_AXIS_COUNT],
                                      vertical_thrust, measured_yaw, dt_s)
        yaw_rate = heading_rate(self.heading_rad, measured_yaw, self.codec.max_rates_rad_s[WORLD_Z_AXIS])
        roll, pitch = output.roll_rad, output.pitch_rad
        if self.flight_mode == 'acro':
            # Acro stabilization only controls rates; navigation closes the tilt loop upstream.
            rates = self.attitude_controller.rates((roll, pitch, measured_yaw), self._feedback.read(observation))
            roll, pitch = rates[:XY_AXIS_COUNT]
        return self.codec.encode(mode=self.flight_mode, rpy=(roll, pitch, yaw_rate),
                                 thrust_n=output.thrust_n, armed=True)
