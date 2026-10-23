"""Altitude → vertical velocity → acceleration → collective thrust (SI units)."""

from dataclasses import dataclass

from bullet_drone.commands import InvalidCommandError, _finite
from bullet_drone.config import ConfigurationError, _number
from bullet_drone.controllers.pid import PID, PidSettings, clamp


@dataclass(frozen=True, slots=True)
class AltitudeControlOutput:
    velocity_target_m_s: float
    acceleration_m_s2: float
    thrust_n: float


class AltitudeController:
    """Level-flight cascade; world Z up. No attitude or mission timing logic.

    Position PID outputs m/s; velocity PID outputs m/s². Gravity compensation
    becomes body +Z thrust for a level vehicle; tilt compensation is not modeled.
    """

    def __init__(self, mass_kg: float, gravity_m_s2: float, max_thrust_n: float,
                 position_pid: PidSettings, velocity_pid: PidSettings) -> None:
        self.mass_kg = _number(mass_kg, "altitude.mass_kg", positive=True)
        self.gravity_m_s2 = _number(gravity_m_s2, "altitude.gravity_m_s2", positive=True)
        self.max_thrust_n = _number(max_thrust_n, "altitude.max_thrust_n", positive=True)
        if self.max_thrust_n <= self.mass_kg * self.gravity_m_s2:
            raise ConfigurationError("Altitude control requires thrust capacity greater than weight")
        self.position = PID(position_pid)
        self.velocity = PID(velocity_pid)
        self.last_output: AltitudeControlOutput | None = None

    def reset(self) -> None:
        self.last_output = None
        self.position.reset()
        self.velocity.reset()

    def thrust(self, target_altitude_m: float, altitude_m: float,
               vertical_velocity_m_s: float, dt_s: float,
               velocity_limit_m_s: float | None = None) -> float:
        # Validate the whole request before advancing either PID.
        target = _finite(target_altitude_m, "target_altitude_m")
        altitude = _finite(altitude_m, "altitude_m")
        velocity = _finite(vertical_velocity_m_s, "vertical_velocity_m_s")
        dt = _finite(dt_s, "dt_s")
        limit = self.position.settings.output_limit if velocity_limit_m_s is None else _finite(velocity_limit_m_s, "velocity_limit_m_s")
        if dt <= 0 or limit <= 0:
            raise InvalidCommandError("dt_s and velocity_limit_m_s must be positive")
        desired_velocity = clamp(self.position.update(target, altitude, dt), limit)
        acceleration = self.velocity.update(desired_velocity, velocity, dt)
        thrust = _finite(self.mass_kg * (self.gravity_m_s2 + acceleration), "altitude thrust")
        bounded_thrust = min(self.max_thrust_n, max(0.0, thrust))
        self.last_output = AltitudeControlOutput(desired_velocity, acceleration, bounded_thrust)
        return bounded_thrust
