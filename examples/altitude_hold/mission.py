"""Example-only mission state machine; times are simulated seconds, Z is world up."""

from dataclasses import dataclass
from enum import StrEnum

from bullet_drone.commands import RcInput
from bullet_drone.controllers.altitude import AltitudeController
from bullet_drone.models import DroneObservation
from bullet_drone.rc import RcCodec


WORLD_Z_AXIS = 2
TARGET_ALTITUDE_M = 3.0
LANDING_ALTITUDE_M = 0.0
GROUND_SETTLE_DURATION_S = 0.5
TAKEOFF_ALTITUDE_TOLERANCE_M = 0.05
TAKEOFF_SPEED_TOLERANCE_M_S = 0.1
TAKEOFF_SETTLE_DURATION_S = 1.0
HOLD_DURATION_S = 10.0
CLIMB_SPEED_LIMIT_M_S = 1.0
DESCENT_SPEED_LIMIT_M_S = 0.5
TOUCHDOWN_ALTITUDE_M = 0.02
TOUCHDOWN_SPEED_TOLERANCE_M_S = 0.05
TOUCHDOWN_SETTLE_DURATION_S = 0.5
DISARMED_SETTLE_DURATION_S = 1.0
DISARMED_THRUST_N = 0.0
LEVEL_RPY_RAD = (0.0, 0.0, 0.0)


class Phase(StrEnum):
    SETTLE = 'settle'
    TAKEOFF = 'takeoff'
    HOLD = 'hold'
    LAND = 'land'
    DISARMED = 'disarmed'
    COMPLETE = 'complete'


@dataclass(frozen=True, slots=True)
class Transition:
    phase: Phase
    simulation_time_s: float


class AltitudeHoldMission:
    """Altitude cascade supplies N to RC encoding; angle mode keeps the drone level.

    Uses ideal position_m[2] (m) and linear_velocity_m_s[2] (m/s), never sensors.
    No XY position control, tilt compensation, or absolute yaw control.
    """

    def __init__(self, altitude: AltitudeController, codec: RcCodec) -> None:
        self.altitude = altitude
        self.codec = codec
        self.phase = Phase.SETTLE
        self.transitions: list[Transition] = []
        self._phase_started_s = 0.0
        self._dwell_started_s: float | None = None

    def reset(self, observation: DroneObservation) -> None:
        self.altitude.reset()
        self.phase = Phase.SETTLE
        self._phase_started_s = observation.ground_truth.simulation_time_s
        self._dwell_started_s = None
        self.transitions = [Transition(self.phase, self._phase_started_s)]

    def _transition(self, phase: Phase, now: float) -> None:
        self.phase = phase
        self._phase_started_s = now
        self._dwell_started_s = None
        self.transitions.append(Transition(phase, now))
        if phase in (Phase.TAKEOFF, Phase.LAND, Phase.DISARMED):
            self.altitude.reset()

    def _settled(self, condition: bool, now: float, duration_s: float) -> bool:
        if not condition:
            self._dwell_started_s = None
        elif self._dwell_started_s is None:
            self._dwell_started_s = now
        return (self._dwell_started_s is not None
                and now - self._dwell_started_s >= duration_s)

    def command(self, observation: DroneObservation, dt_s: float) -> RcInput:
        state = observation.ground_truth
        now = state.simulation_time_s
        altitude = state.position_m[WORLD_Z_AXIS]
        speed = state.linear_velocity_m_s[WORLD_Z_AXIS]

        if self.phase == Phase.SETTLE and now - self._phase_started_s >= GROUND_SETTLE_DURATION_S:
            self._transition(Phase.TAKEOFF, now)
        elif self.phase == Phase.TAKEOFF:
            if self._settled(
                abs(altitude - TARGET_ALTITUDE_M) <= TAKEOFF_ALTITUDE_TOLERANCE_M
                and abs(speed) <= TAKEOFF_SPEED_TOLERANCE_M_S,
                now, TAKEOFF_SETTLE_DURATION_S,
            ):
                self._transition(Phase.HOLD, now)
        elif self.phase == Phase.HOLD and now - self._phase_started_s >= HOLD_DURATION_S:
            self._transition(Phase.LAND, now)
        elif self.phase == Phase.LAND:
            if self._settled(
                altitude <= TOUCHDOWN_ALTITUDE_M and abs(speed) <= TOUCHDOWN_SPEED_TOLERANCE_M_S,
                now, TOUCHDOWN_SETTLE_DURATION_S,
            ):
                self._transition(Phase.DISARMED, now)
        elif self.phase == Phase.DISARMED and now - self._phase_started_s >= DISARMED_SETTLE_DURATION_S:
            self._transition(Phase.COMPLETE, now)

        armed = self.phase in (Phase.TAKEOFF, Phase.HOLD, Phase.LAND)
        thrust = DISARMED_THRUST_N
        if armed:
            landing = self.phase == Phase.LAND
            thrust = self.altitude.thrust(
                LANDING_ALTITUDE_M if landing else TARGET_ALTITUDE_M, altitude, speed, dt_s,
                velocity_limit_m_s=DESCENT_SPEED_LIMIT_M_S if landing else CLIMB_SPEED_LIMIT_M_S,
            )
        return self.codec.encode(mode='angle', rpy=LEVEL_RPY_RAD,
                                 thrust_n=thrust, armed=armed)
