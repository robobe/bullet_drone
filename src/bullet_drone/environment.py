"""Simulation lifecycle and observations, independent of a physics library."""

from bullet_drone.attitude import AttitudeEstimator
from bullet_drone.commands import InvalidCommandError, RotorCommand
from bullet_drone.contracts import LifecycleError, PhysicsBackend, Sensor
from bullet_drone.models import BarometerReading, DroneObservation, DroneState, ImuReading, SensorReadings


class DroneEnvironment:
    def __init__(self, physics: PhysicsBackend, *, imu: Sensor[ImuReading] | None = None,
                 barometer: Sensor[BarometerReading] | None = None,
                 attitude_estimator: AttitudeEstimator | None = None) -> None:
        self._physics = physics
        self._imu = imu
        self._barometer = barometer
        self._attitude_estimator = attitude_estimator
        self._ready = False
        self._closed = False
        self._state = None

    def reset(self) -> DroneObservation:
        if self._closed:
            raise LifecycleError("Cannot reset a closed environment")
        self._ready = False
        state = self._physics.reset()
        observation = self._observe(state, reset=True)
        self._state = state
        self._ready = True
        return observation

    def step(self, action: RotorCommand | None = None) -> DroneObservation:
        if self._closed or not self._ready:
            raise LifecycleError("Environment must be reset before stepping and cannot be closed or failed")
        if action is not None:
            if not isinstance(action, RotorCommand):
                raise InvalidCommandError("Action must be a RotorCommand or None")
            action.validate_for(self._state.episode_id, self._state.step_index + 1)
        try:
            state = self._physics.step() if action is None else self._physics.step(action)
        except InvalidCommandError:
            # Backend limit checks run before any force application or stepping.
            raise
        except BaseException:
            self._ready = False
            raise

        try:
            observation = self._observe(state)
            self._state = state
            return observation
        except BaseException:
            self._ready = False
            raise

    def _observe(self, state: DroneState, *, reset: bool = False) -> DroneObservation:
        imu = (self._imu.reset(state) if reset else self._imu.sample(state)) if self._imu else None
        barometer = ((self._barometer.reset(state) if reset else self._barometer.sample(state))
                     if self._barometer else None)
        estimate = None
        if self._attitude_estimator is not None:
            estimate = (self._attitude_estimator.reset(imu, state.orientation_xyzw) if reset
                        else self._attitude_estimator.update(imu))
        return DroneObservation(state, SensorReadings(imu, barometer), estimate)

    def close(self) -> None:
        self._ready = False
        self._closed = True
        self._physics.close()
