"""Horizontal velocity PID and world-force → body-angle conversion (SI units)."""

from dataclasses import dataclass
from math import asin, atan2, cos, hypot, radians, sin, tan

from bullet_drone.commands import InvalidCommandError, _finite, _values
from bullet_drone.config import ConfigurationError, _number
from bullet_drone.controllers.pid import PID, PidSettings


XY_AXIS_COUNT = 2
MAX_SUPPORTED_TILT_RAD = radians(60)


@dataclass(frozen=True, slots=True)
class VelocityControlOutput:
    """World XY acceleration (m/s²), ZYX roll/pitch (rad), body +Z thrust (N)."""

    acceleration_xy_m_s2: tuple[float, float]
    roll_rad: float
    pitch_rad: float
    thrust_n: float


class VelocityController:
    """Independent velocity PIDs produce acceleration, then bounded tilt/thrust.

    Vertical thrust is supplied by altitude control. Rotate horizontal force
    using measured world yaw; include tilt compensation in collective thrust.
    Acceleration limits apply per axis; tilt limits the combined force vector.
    At motor capacity the collective clips, reducing available vertical force.
    This stage knows no RC encoding, yaw-heading control, or mission timing.
    """

    def __init__(self, mass_kg: float, max_thrust_n: float,
                 velocity_pid: PidSettings, max_tilt_rad: float) -> None:
        self.mass_kg = _number(mass_kg, 'velocity.mass_kg', positive=True)
        self.max_thrust_n = _number(max_thrust_n, 'velocity.max_thrust_n', positive=True)
        self.max_tilt_rad = _number(max_tilt_rad, 'velocity.max_tilt_rad', positive=True)
        if self.max_tilt_rad >= MAX_SUPPORTED_TILT_RAD:
            raise ConfigurationError('velocity.max_tilt_rad must be less than 60 degrees')
        self._pids = tuple(PID(velocity_pid) for _ in range(XY_AXIS_COUNT))
        self.last_output: VelocityControlOutput | None = None

    def reset(self) -> None:
        self.last_output = None
        for pid in self._pids:
            pid.reset()

    def update(self, target_velocity_xy_m_s: tuple[float, float],
               velocity_xy_m_s: tuple[float, float], vertical_thrust_n: float,
               measured_yaw_rad: float, dt_s: float) -> VelocityControlOutput:
        target = _values(target_velocity_xy_m_s, XY_AXIS_COUNT, 'target_velocity_xy_m_s')
        velocity = _values(velocity_xy_m_s, XY_AXIS_COUNT, 'velocity_xy_m_s')
        fz = _finite(vertical_thrust_n, 'vertical_thrust_n')
        yaw = _finite(measured_yaw_rad, 'measured_yaw_rad')
        dt = _finite(dt_s, 'dt_s')
        if fz < 0 or dt <= 0:
            raise InvalidCommandError('vertical_thrust_n must be nonnegative and dt_s positive')
        accelerations = tuple(pid.update(t, v, dt) for pid, t, v in zip(self._pids, target, velocity))
        fx, fy = (_finite(a * self.mass_kg, 'horizontal force') for a in accelerations)
        horizontal = hypot(fx, fy)
        allowed = fz * tan(self.max_tilt_rad)
        factor = allowed / horizontal if horizontal > allowed else 1.0
        fx, fy = fx * factor, fy * factor
        thrust = hypot(fx, fy, fz)
        roll = asin(max(-1.0, min(1.0, (fx * sin(yaw) - fy * cos(yaw)) / thrust))) if thrust else 0.0
        pitch = atan2(fx * cos(yaw) + fy * sin(yaw), fz)
        self.last_output = VelocityControlOutput(accelerations, roll, pitch,
                                                min(thrust, self.max_thrust_n))
        return self.last_output
