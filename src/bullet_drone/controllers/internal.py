"""Attitude P → body-rate PID → requested body wrench; no physics integration."""

from dataclasses import dataclass

from bullet_drone.commands import HighLevelCommand, InvalidCommandError, WrenchCommand, _finite, _values
from bullet_drone.config import ConfigurationError, _vector
from bullet_drone.geometry import GeometryError, attitude_error, from_rpy, normalize, world_to_body, yaw_from_quaternion
from bullet_drone.controllers.pid import PID, PidSettings, clamp
from bullet_drone.models import ControlFeedback, DroneObservation, Vector3


@dataclass(frozen=True, slots=True)
class InternalControlSettings:
    attitude_kp: Vector3  # (rad/s)/rad
    max_rates_rad_s: Vector3
    rate_pid: tuple[PidSettings, PidSettings, PidSettings]  # outputs N*m
    rate_feedforward: Vector3 = (0.0, 0.0, 0.0)  # N*m/(rad/s)

    def __post_init__(self) -> None:
        for name in ("attitude_kp", "max_rates_rad_s", "rate_feedforward"):
            values = _vector(getattr(self, name), name)
            if any(v < 0 or (name == "max_rates_rad_s" and v == 0) for v in values):
                raise ConfigurationError(f"{name} requires {'positive' if name == 'max_rates_rad_s' else 'nonnegative'} values")
            object.__setattr__(self, name, values)
        if (not isinstance(self.rate_pid, (tuple, list)) or len(self.rate_pid) != 3
                or any(not isinstance(s, PidSettings) for s in self.rate_pid)):
            raise ConfigurationError("rate_pid requires three PidSettings")
        object.__setattr__(self, "rate_pid", tuple(self.rate_pid))


class IdealStateFeedback:
    """Read-only ground-truth adapter; not a simulated IMU or noisy sensor."""

    def read(self, observation: DroneObservation) -> ControlFeedback:
        state = observation.ground_truth
        orientation = normalize(state.orientation_xyzw)
        return ControlFeedback(orientation, world_to_body(state.angular_velocity_rad_s, orientation))


class AttitudeController:
    def __init__(self, kp: Vector3, max_rates_rad_s: Vector3) -> None:
        self.kp = kp
        self.limits = max_rates_rad_s

    def rates(self, rpy: Vector3, feedback: ControlFeedback) -> Vector3:
        try:
            error = attitude_error(feedback.orientation_xyzw, from_rpy(rpy))
        except GeometryError as exc:
            raise InvalidCommandError(str(exc)) from exc
        return tuple(clamp(k * e, limit) for k, e, limit in zip(self.kp, error, self.limits))


class RateController:
    def __init__(self, settings: InternalControlSettings) -> None:
        self.settings = settings
        self.pids = tuple(PID(s) for s in settings.rate_pid)

    def reset(self) -> None:
        for pid in self.pids:
            pid.reset()

    def torque(self, targets: Vector3, measurements: Vector3, dt_s: float) -> Vector3:
        return tuple(pid.update(target, measurement, dt_s, ff * target)
                     for pid, target, measurement, ff in zip(self.pids, targets, measurements, self.settings.rate_feedforward))


class InternalFlightController:
    def __init__(self, settings: InternalControlSettings) -> None:
        self.settings = settings
        self.attitude = AttitudeController(settings.attitude_kp, settings.max_rates_rad_s)
        self.rate = RateController(settings)
        self._mode: str | None = None

    def reset(self) -> None:
        self.rate.reset()
        self._mode = None

    def update(self, command: HighLevelCommand, feedback: ControlFeedback, dt_s: float) -> WrenchCommand:
        dt_s = _finite(dt_s, "dt_s")
        if dt_s <= 0:
            raise InvalidCommandError("dt_s must be positive")
        # Validate all feedback before mutating any PID history.
        try:
            normalize(feedback.orientation_xyzw)
            measurements = _values(feedback.body_rates_rad_s, 3, "body_rates_rad_s")
            if command.mode == "angle":
                rpy = command.rpy
                if command.yaw_rate_rad_s is not None:
                    # Tilt stabilization follows current heading; yaw is directly rate-controlled.
                    rpy = (*rpy[:2], yaw_from_quaternion(feedback.orientation_xyzw))
                targets = self.attitude.rates(rpy, feedback)
                if command.yaw_rate_rad_s is not None:
                    targets = (*targets[:2], clamp(command.yaw_rate_rad_s, self.settings.max_rates_rad_s[2]))
            else:
                targets = tuple(clamp(v, limit) for v, limit in zip(command.rpy, self.settings.max_rates_rad_s))
        except GeometryError as exc:
            raise InvalidCommandError(str(exc)) from exc
        if command.mode != self._mode:
            self.rate.reset()
            self._mode = command.mode
        return WrenchCommand(command.thrust_n, self.rate.torque(targets, measurements, dt_s))
