"""Small backend-independent lifecycle contracts."""

from typing import Protocol

from bullet_drone.commands import HighLevelCommand, RotorCommand, WrenchCommand
from bullet_drone.models import ControlFeedback, DroneObservation, DroneState, PropellerOutput, RotorSpeeds, SurfaceHit, Vector3


class LifecycleError(RuntimeError):
    """An operation was requested before reset or after close/failure."""


class BackendError(RuntimeError):
    """A physics operation failed; the current episode cannot continue."""


class PhysicsBackend(Protocol):
    def reset(self) -> DroneState: ...
    def step(self, command: RotorCommand | None = None) -> DroneState: ...
    def close(self) -> None: ...


class Sensor[Reading](Protocol):
    """Read-only state to measurements; reset restarts history and seeded noise."""

    def reset(self, state: DroneState) -> Reading: ...
    def sample(self, state: DroneState) -> Reading: ...


class HighLevelController(Protocol):
    def reset(self, observation: DroneObservation) -> None: ...
    def command(self, observation: DroneObservation, dt_s: float) -> HighLevelCommand: ...
    def close(self) -> None: ...


class FeedbackSource(Protocol):
    def read(self, observation: DroneObservation) -> ControlFeedback: ...


class StabilizationController(Protocol):
    def reset(self) -> None: ...
    def update(self, command: HighLevelCommand, feedback: ControlFeedback, dt_s: float) -> WrenchCommand: ...


class MotorDynamics(Protocol):
    """Four prop0–prop3 actual rad/s; reset stops motors, update uses positive dt."""

    def reset(self) -> RotorSpeeds: ...
    def update(self, targets_rad_s: RotorSpeeds, dt_s: float) -> RotorSpeeds: ...


class PropellerModel(Protocol):
    """Actual rad/s to body-frame forces and reaction torque; no integration."""

    def calculate(self, speeds_rad_s: RotorSpeeds) -> PropellerOutput: ...


class ObservationConsumer(Protocol):
    """Read-only completed observations; caller owns lifecycle."""
    def reset(self, observation: DroneObservation) -> None: ...
    def consume(self, observation: DroneObservation) -> None: ...
    def close(self) -> None: ...


class SurfaceQueries(Protocol):
    """World -Z supporting surface queries; exclude the owning drone and its links."""

    def surface_below(self, origin_world_m: Vector3, range_m: float) -> SurfaceHit | None: ...
