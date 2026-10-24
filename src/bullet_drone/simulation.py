"""One RC request per physics step; the caller owns the execution loop."""

from bullet_drone.attitude import AttitudeEstimator
from bullet_drone.commands import HighLevelCommand, InvalidCommandError, RcInput, MotorAllocation
from bullet_drone.composition import FlightCommandProvider, validate_control_backend
from bullet_drone.config import SimulationConfig
from bullet_drone.contracts import LifecycleError, MotorDynamics, PropellerModel, StabilizationController
from bullet_drone.controllers import IdealStateFeedback, InternalControlSettings, InternalFlightController
from bullet_drone.environment import DroneEnvironment
from bullet_drone.rc import RcCodec, SoftwareRc
from bullet_drone.sensors import BarometerSensor, ImuSensor
from bullet_drone.mixer import MotorMixer
from bullet_drone.models import DroneObservation
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.vehicle import VehicleParameters, load_racer_parameters


class RcSimulation:
    """Own RC translation, stabilization and physics, but no run loop.

    Inputs contain 18 integer RC channels in 1000..2000. Outputs
    contain immutable ground truth and configured sensor readings. Optional
    component injections are library contracts for models/probes, not alternative
    per-step command paths.
    Runtime failures require reset before another step; close is terminal.
    """

    def __init__(self, config: SimulationConfig, control_settings: InternalControlSettings,
                 vehicle: VehicleParameters | None = None, *,
                 stabilization: StabilizationController | None = None,
                 motor_dynamics: MotorDynamics | None = None,
                 propeller_model: PropellerModel | None = None) -> None:
        validate_control_backend(config.control)
        vehicle = vehicle if vehicle is not None else load_racer_parameters()
        self._timestep_s = config.simulation.timestep_s
        self._rc = SoftwareRc(RcCodec(4 * vehicle.max_thrust_per_rotor_n,
                                       control_settings.max_rates_rad_s, config.rc))
        self._provider = FlightCommandProvider(
            self._rc,
            stabilization if stabilization is not None else InternalFlightController(control_settings),
            IdealStateFeedback(), MotorMixer(vehicle), self._timestep_s,
        )
        self._physics = PyBulletPhysics(config, vehicle, motor_dynamics=motor_dynamics, propeller_model=propeller_model)
        self._environment = DroneEnvironment(self._physics, imu=(ImuSensor(config.simulation.gravity_m_s2, config.sensors.imu, sample_period_s=self._timestep_s)
                if config.sensors.imu.enabled else None),
            barometer=(BarometerSensor(config.sensors.barometer)
                       if config.sensors.barometer.enabled else None),
            attitude_estimator=(AttitudeEstimator(config.simulation.gravity_m_s2, config.attitude_estimator)
                                if config.pose_sources.attitude == 'imu' else None))
        self._observation: DroneObservation | None = None
        self._last_command: HighLevelCommand | None = None
        self._closed = False

    @property
    def wind(self):
        return self._physics.wind

    def set_wind(self, settings):
        if self._closed or self._observation is None:
            raise LifecycleError('Simulation must be reset before updating wind')
        self._physics.set_wind(settings)

    @property
    def timestep_s(self) -> float:
        return self._timestep_s

    @property
    def last_command(self) -> HighLevelCommand | None:
        """Read-only identity-stamped command for the last completed step."""
        return self._last_command

    @property
    def allocation(self) -> MotorAllocation | None:
        """Read-only allocation for the last completed armed step."""
        return self._provider.allocation if self._last_command is not None else None

    def reset(self) -> DroneObservation:
        if self._closed:
            raise LifecycleError("Cannot reset a closed RC simulation")
        self._observation = None
        self._last_command = None
        observation = self._environment.reset()
        self._provider.reset(observation)
        self._observation = observation
        return observation

    def step(self, command: RcInput) -> DroneObservation:
        if self._closed or self._observation is None:
            raise LifecycleError("RC simulation must be reset before stepping")
        if not isinstance(command, RcInput):
            raise InvalidCommandError("Simulation requires an RcInput")
        # Revalidate even if a frozen input has been deliberately tampered with.
        value = RcInput(command.channels)
        feedback = self._observation
        self._last_command = None
        try:
            self._rc.set_input(value)
            rotor_command = self._provider.command(feedback)
            observation = self._environment.step(rotor_command)
        except BaseException:
            # Translation may have consumed the step; never retry it implicitly.
            self._observation = None
            raise
        self._last_command = self._rc.last_command
        self._observation = observation
        return observation

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._observation = None
        failure = None
        for resource in (self._provider, self._environment):
            try:
                resource.close()
            except BaseException as exc:
                if failure is None:
                    failure = exc
                else:
                    failure.add_note(f"Additional simulation cleanup failure: {exc}")
        if failure is not None:
            raise failure
