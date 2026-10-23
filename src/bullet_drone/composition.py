"""Controller selection and API-only composition; no simulation resources opened."""

from bullet_drone.commands import HighLevelCommand, InvalidCommandError, RotorCommand
from bullet_drone.config import ConfigurationError, ControlSettings, _number
from bullet_drone.contracts import FeedbackSource, HighLevelController, LifecycleError, StabilizationController
from bullet_drone.controllers import IdealStateFeedback, InternalControlSettings, InternalFlightController
from bullet_drone.mixer import MotorMixer
from bullet_drone.models import DroneObservation
from bullet_drone.vehicle import VehicleParameters


class UnsupportedControllerError(ConfigurationError):
    """A recognized controller backend is not implemented."""


def validate_control_backend(settings: ControlSettings) -> None:
    if settings.backend == "betaflight":
        raise UnsupportedControllerError("Betaflight control is not implemented; use control.backend: internal")


class FlightCommandProvider:
    """Bridge high-level commands to bounded motor targets for one episode.

    Caller owns reset/close. Invalid commands do not advance stabilization;
    runtime component failures require reset before reuse.
    """

    def __init__(self, source: HighLevelController, controller: StabilizationController,
                 feedback: FeedbackSource, mixer: MotorMixer, dt_s: float) -> None:
        self._source = source
        self._controller = controller
        self._feedback = feedback
        self._mixer = mixer
        self._dt_s = _number(dt_s, "control.dt_s", positive=True)
        self._episode_id: str | None = None
        self._next_step = 0
        self._closed = False
        self._armed: bool | None = None
        self.allocation = None

    def reset(self, observation: DroneObservation) -> None:
        if self._closed:
            raise LifecycleError("Cannot reset a closed command provider")
        self._episode_id = None
        self.allocation = None
        self._armed = None
        self._controller.reset()
        self._source.reset(observation)
        state = observation.ground_truth
        self._episode_id = state.episode_id
        self._next_step = state.step_index + 1

    def command(self, observation: DroneObservation) -> RotorCommand:
        state = observation.ground_truth
        if self._closed or self._episode_id is None:
            raise LifecycleError("Command provider must be reset before use")
        if state.episode_id != self._episode_id or state.step_index + 1 != self._next_step:
            raise InvalidCommandError("Observation is stale or belongs to another episode/step")
        try:
            command = self._source.command(observation, self._dt_s)
            if not isinstance(command, HighLevelCommand):
                raise InvalidCommandError("High-level controller requires a HighLevelCommand")
            command.validate_for(state.episode_id, state.step_index + 1)
        except InvalidCommandError:
            raise
        except BaseException:
            self._episode_id = None
            raise
        try:
            if command.armed != self._armed:
                self._controller.reset()
            if command.armed:
                feedback = self._feedback.read(observation)
                wrench = self._controller.update(command, feedback, self._dt_s)
                allocation = self._mixer.mix(wrench)
                speeds = allocation.speeds_rad_s
            else:
                allocation = None
                speeds = (0.0, 0.0, 0.0, 0.0)
            result = RotorCommand(speeds, state.episode_id, state.step_index + 1)
        except BaseException:
            self._episode_id = None
            raise
        self._armed = command.armed
        self.allocation = allocation
        self._next_step += 1
        return result

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._episode_id = None
        self._source.close()


def create_flight_command_provider(control: ControlSettings, source: HighLevelController,
                                   settings: InternalControlSettings, vehicle: VehicleParameters,
                                   dt_s: float, feedback: FeedbackSource | None = None) -> FlightCommandProvider:
    validate_control_backend(control)
    return FlightCommandProvider(source, InternalFlightController(settings),
                                 feedback if feedback is not None else IdealStateFeedback(),
                                 MotorMixer(vehicle), dt_s)
