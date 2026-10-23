"""Load and validate simulation configuration before acquiring resources."""

from dataclasses import dataclass, field, fields
from math import hypot, isfinite, pi
from pathlib import Path

import yaml

from bullet_drone.models import DEFAULT_DRONE_ID, MAX_DRONE_ID_LENGTH, Vector3


ZERO_VECTOR = (0.0, 0.0, 0.0)
DEFAULT_IMU_SEED = 0
DEFAULT_BAROMETER_SEED = 1
DEFAULT_REFERENCE_PRESSURE_PA = 101325.0
DEFAULT_REFERENCE_TEMPERATURE_K = 288.15
DEFAULT_REFERENCE_ALTITUDE_M = 0.0

DEFAULT_AIR_DENSITY_KG_M3 = 1.225
DEFAULT_ROTOR_RADIUS_M = 0.0635
DEFAULT_BODY_CD_AREA_M2 = (0.02, 0.02, 0.03)
DEFAULT_ROTOR_DRAG_COEFFICIENTS = (1e-5, 1e-5, 1.1e-5)
DEFAULT_ANGULAR_DAMPING_NM_PER_RAD_S = (0.002, 0.002, 0.001)
DEFAULT_INFLOW_COEFFICIENT = 1.0
DEFAULT_MINIMUM_INFLOW_MULTIPLIER = 0.5
DEFAULT_FLAPPING_COEFFICIENT = 0.1
DEFAULT_ROTOR_INERTIA_KG_M2 = 1e-5
DEFAULT_GROUND_EFFECT_COEFFICIENT = 0.1
DEFAULT_GROUND_EFFECT_MAXIMUM = 1.2
DEFAULT_GROUND_QUERY_RANGE_M = 1.0
DEFAULT_GROUND_MINIMUM_DISTANCE_M = 0.01
DEFAULT_WAKE_SPREAD = 0.1
DEFAULT_WAKE_MAXIMUM_SPEED_M_S = 20.0
DEFAULT_WAKE_BODY_SAMPLE_OFFSET_M = (0.0, 0.0, -0.03)
DEFAULT_WIND_CORRELATION_TIME_S = 1.0
DEFAULT_WIND_SEED = 2


class ConfigurationError(ValueError):
    """The configuration cannot describe a valid simulation."""


def _number(value: object, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigurationError(f"{name} must be a finite number")
    try:
        result = float(value)
    except OverflowError as exc:
        raise ConfigurationError(f"{name} must be a finite number") from exc
    if not isfinite(result) or (positive and result <= 0):
        qualifier = "positive and finite" if positive else "finite"
        raise ConfigurationError(f"{name} must be {qualifier}")
    return result


def _vector(value: object, name: str) -> Vector3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ConfigurationError(f"{name} must contain exactly three numbers")
    return tuple(_number(item, f"{name}[{i}]") for i, item in enumerate(value))


@dataclass(frozen=True, slots=True)
class SimulationSettings:
    timestep_s: float
    max_steps: int
    gravity_m_s2: Vector3
    gui: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestep_s", _number(self.timestep_s, "simulation.timestep_s", positive=True))
        if type(self.max_steps) is not int or self.max_steps <= 0:
            raise ConfigurationError("simulation.max_steps must be a positive integer")
        object.__setattr__(self, "gravity_m_s2", _vector(self.gravity_m_s2, "simulation.gravity_m_s2"))
        if type(self.gui) is not bool:
            raise ConfigurationError("simulation.gui must be a boolean")


@dataclass(frozen=True, slots=True)
class DroneSettings:
    mass_kg: float
    initial_position_m: Vector3
    drone_id: str = DEFAULT_DRONE_ID

    def __post_init__(self) -> None:
        if (not isinstance(self.drone_id, str) or not self.drone_id.strip()
                or len(self.drone_id) > MAX_DRONE_ID_LENGTH):
            raise ConfigurationError(
                f"drone.drone_id must be a nonblank string of at most {MAX_DRONE_ID_LENGTH} characters"
            )
        object.__setattr__(self, "mass_kg", _number(self.mass_kg, "drone.mass_kg", positive=True))
        object.__setattr__(self, "initial_position_m", _vector(self.initial_position_m, "drone.initial_position_m"))


@dataclass(frozen=True, slots=True)
class ControlSettings:
    backend: str = "internal"

    def __post_init__(self) -> None:
        if not isinstance(self.backend, str) or self.backend not in ("internal", "betaflight"):
            raise ConfigurationError("control.backend must be 'internal' or 'betaflight'")


@dataclass(frozen=True, slots=True)
class RcSettings:
    """One-based AETR channel assignments, AUX switches and roll/pitch angle scale."""

    aetr_channels: tuple[int, int, int, int] = (1, 2, 3, 4)
    arm_channel: int = 5
    mode_channel: int = 6
    arm_threshold: int = 1500
    angle_threshold: int = 1500
    max_angle_rad: float = pi / 4

    def __post_init__(self) -> None:
        channels = self.aetr_channels
        if not isinstance(channels, (tuple, list)) or len(channels) != 4:
            raise ConfigurationError("rc.aetr_channels requires four channel numbers")
        assignments = (*channels, self.arm_channel, self.mode_channel)
        if (any(type(v) is not int for v in assignments)
                or set(channels) != {1, 2, 3, 4}
                or not 5 <= self.arm_channel <= 18 or not 5 <= self.mode_channel <= 18
                or len(set(assignments)) != 6):
            raise ConfigurationError("AETR must permute CH1..4; distinct arm/mode AUX channels must be in CH5..18")
        object.__setattr__(self, 'aetr_channels', tuple(channels))
        for name in ('arm_threshold', 'angle_threshold'):
            value = getattr(self, name)
            if type(value) is not int or not 1000 < value <= 2000:
                raise ConfigurationError(f"rc.{name} must be an integer in 1001..2000")
        angle = _number(self.max_angle_rad, 'rc.max_angle_rad', positive=True)
        if angle >= pi / 2:
            raise ConfigurationError('rc.max_angle_rad must be less than pi/2')
        object.__setattr__(self, 'max_angle_rad', angle)


def _sensor_options(enabled: object, seed: object) -> None:
    if type(enabled) is not bool:
        raise ConfigurationError("sensor enabled must be a boolean")
    if type(seed) is not int:
        raise ConfigurationError("sensor seed must be an integer")


@dataclass(frozen=True, slots=True)
class ImuSettings:
    enabled: bool = True
    accelerometer_stddev_m_s2: Vector3 = ZERO_VECTOR
    accelerometer_bias_m_s2: Vector3 = ZERO_VECTOR
    gyro_stddev_rad_s: Vector3 = ZERO_VECTOR
    gyro_bias_rad_s: Vector3 = ZERO_VECTOR
    seed: int = DEFAULT_IMU_SEED
    noise_model: str = 'per_sample'
    accelerometer_noise_density_m_s2_sqrt_hz: Vector3 = ZERO_VECTOR
    gyro_noise_density_rad_s_sqrt_hz: Vector3 = ZERO_VECTOR
    filter_enabled: bool = False
    accelerometer_cutoff_hz: float = 30.0
    gyro_cutoff_hz: float = 60.0
    accelerometer_bias_random_walk_m_s2_sqrt_s: Vector3 = ZERO_VECTOR
    gyro_bias_random_walk_rad_s_sqrt_s: Vector3 = ZERO_VECTOR

    def __post_init__(self) -> None:
        _sensor_options(self.enabled, self.seed)
        if self.noise_model not in ('per_sample', 'density'):
            raise ConfigurationError('sensors.imu.noise_model must be per_sample or density')
        if type(self.filter_enabled) is not bool:
            raise ConfigurationError('sensors.imu.filter_enabled must be a boolean')
        for name in ("accelerometer_stddev_m_s2", "accelerometer_bias_m_s2",
                     "gyro_stddev_rad_s", "gyro_bias_rad_s",
                     "accelerometer_noise_density_m_s2_sqrt_hz", "gyro_noise_density_rad_s_sqrt_hz",
                     "accelerometer_bias_random_walk_m_s2_sqrt_s", "gyro_bias_random_walk_rad_s_sqrt_s"):
            values = _vector(getattr(self, name), f"sensors.imu.{name}")
            if any(word in name for word in ('stddev', 'density', 'random_walk')) and any(value < 0 for value in values):
                raise ConfigurationError(f"sensors.imu.{name} must be nonnegative")
            object.__setattr__(self, name, values)
        if self.noise_model == 'density' and any((*self.accelerometer_stddev_m_s2, *self.gyro_stddev_rad_s)):
            raise ConfigurationError('density mode cannot use per-sample IMU standard deviations')
        if self.noise_model == 'per_sample' and any((*self.accelerometer_noise_density_m_s2_sqrt_hz,
                                                   *self.gyro_noise_density_rad_s_sqrt_hz)):
            raise ConfigurationError('IMU noise densities require density mode')
        for name in ('accelerometer_cutoff_hz', 'gyro_cutoff_hz'):
            object.__setattr__(self, name, _number(getattr(self, name), f'sensors.imu.{name}', positive=True))

    def validate_sample_period(self, period_s: float) -> float:
        period = _number(period_s, 'imu.sample_period_s', positive=True)
        if self.filter_enabled and any(2 * cutoff * period >= 1 for cutoff in
                                       (self.accelerometer_cutoff_hz, self.gyro_cutoff_hz)):
            raise ConfigurationError('IMU filter cutoffs must be below the sampling Nyquist frequency')
        return period


@dataclass(frozen=True, slots=True)
class BarometerSettings:
    enabled: bool = True
    pressure_stddev_pa: float = 0.0
    pressure_bias_pa: float = 0.0
    reference_pressure_pa: float = DEFAULT_REFERENCE_PRESSURE_PA
    reference_temperature_k: float = DEFAULT_REFERENCE_TEMPERATURE_K
    reference_altitude_m: float = DEFAULT_REFERENCE_ALTITUDE_M
    seed: int = DEFAULT_BAROMETER_SEED
    filter_enabled: bool = False
    filter_time_constant_s: float = 0.1

    def __post_init__(self) -> None:
        _sensor_options(self.enabled, self.seed)
        if type(self.filter_enabled) is not bool:
            raise ConfigurationError('sensors.barometer.filter_enabled must be a boolean')
        for name in ("pressure_stddev_pa", "pressure_bias_pa", "reference_pressure_pa",
                     "reference_temperature_k", "reference_altitude_m", "filter_time_constant_s"):
            value = _number(getattr(self, name), f"sensors.barometer.{name}",
                            positive=name in ("reference_pressure_pa", "reference_temperature_k", "filter_time_constant_s"))
            if name == "pressure_stddev_pa" and value < 0:
                raise ConfigurationError("sensors.barometer.pressure_stddev_pa must be nonnegative")
            object.__setattr__(self, name, value)


@dataclass(frozen=True, slots=True)
class SensorsSettings:
    imu: ImuSettings = field(default_factory=ImuSettings)
    barometer: BarometerSettings = field(default_factory=BarometerSettings)

    def __post_init__(self) -> None:
        if not isinstance(self.imu, ImuSettings) or not isinstance(self.barometer, BarometerSettings):
            raise ConfigurationError("sensors requires ImuSettings and BarometerSettings")


def _nonnegative(value: object, name: str) -> float:
    result = _number(value, name)
    if result < 0:
        raise ConfigurationError(f"{name} must be nonnegative")
    return result


def _effect_values(settings, *, scalars=(), vectors=()) -> None:
    if type(settings.enabled) is not bool:
        raise ConfigurationError("effect enabled must be a boolean")
    for name in scalars:
        object.__setattr__(settings, name, _nonnegative(getattr(settings, name), name))
    for name in vectors:
        values = _vector(getattr(settings, name), name)
        if any(value < 0 for value in values):
            raise ConfigurationError(f"{name} must be nonnegative")
        object.__setattr__(settings, name, values)


@dataclass(frozen=True, slots=True)
class BodyDragSettings:
    enabled: bool = True
    cd_area_m2: Vector3 = DEFAULT_BODY_CD_AREA_M2

    def __post_init__(self) -> None:
        _effect_values(self, vectors=("cd_area_m2",))


@dataclass(frozen=True, slots=True)
class RotorDragSettings:
    enabled: bool = True
    coefficients_n_per_rad_s_m_s: Vector3 = DEFAULT_ROTOR_DRAG_COEFFICIENTS

    def __post_init__(self) -> None:
        _effect_values(self, vectors=("coefficients_n_per_rad_s_m_s",))


@dataclass(frozen=True, slots=True)
class AngularDampingSettings:
    enabled: bool = True
    coefficients_nm_per_rad_s: Vector3 = DEFAULT_ANGULAR_DAMPING_NM_PER_RAD_S

    def __post_init__(self) -> None:
        _effect_values(self, vectors=("coefficients_nm_per_rad_s",))


@dataclass(frozen=True, slots=True)
class InflowSettings:
    enabled: bool = True
    coefficient: float = DEFAULT_INFLOW_COEFFICIENT
    minimum_multiplier: float = DEFAULT_MINIMUM_INFLOW_MULTIPLIER

    def __post_init__(self) -> None:
        _effect_values(self, scalars=("coefficient", "minimum_multiplier"))
        if not 0 < self.minimum_multiplier <= 1:
            raise ConfigurationError("minimum_multiplier must be in (0, 1]")


@dataclass(frozen=True, slots=True)
class FlappingSettings:
    enabled: bool = True
    coefficient: float = DEFAULT_FLAPPING_COEFFICIENT

    def __post_init__(self) -> None:
        _effect_values(self, scalars=("coefficient",))


@dataclass(frozen=True, slots=True)
class GyroscopicSettings:
    enabled: bool = True
    rotor_inertia_kg_m2: float = DEFAULT_ROTOR_INERTIA_KG_M2

    def __post_init__(self) -> None:
        _effect_values(self, scalars=("rotor_inertia_kg_m2",))


@dataclass(frozen=True, slots=True)
class GroundEffectSettings:
    enabled: bool = True
    coefficient: float = DEFAULT_GROUND_EFFECT_COEFFICIENT
    maximum_multiplier: float = DEFAULT_GROUND_EFFECT_MAXIMUM
    query_range_m: float = DEFAULT_GROUND_QUERY_RANGE_M
    minimum_distance_m: float = DEFAULT_GROUND_MINIMUM_DISTANCE_M

    def __post_init__(self) -> None:
        _effect_values(self, scalars=("coefficient", "maximum_multiplier"))
        for name in ("query_range_m", "minimum_distance_m"):
            object.__setattr__(self, name, _number(getattr(self, name), name, positive=True))
        if self.maximum_multiplier < 1 or self.minimum_distance_m > self.query_range_m:
            raise ConfigurationError("ground effect requires maximum_multiplier >= 1 and minimum_distance_m <= query_range_m")


@dataclass(frozen=True, slots=True)
class PropwashSettings:
    enabled: bool = True
    spread: float = DEFAULT_WAKE_SPREAD
    maximum_speed_m_s: float = DEFAULT_WAKE_MAXIMUM_SPEED_M_S
    body_sample_offset_m: Vector3 = DEFAULT_WAKE_BODY_SAMPLE_OFFSET_M

    def __post_init__(self) -> None:
        _effect_values(self, scalars=("spread",))
        object.__setattr__(self, "maximum_speed_m_s",
                           _number(self.maximum_speed_m_s, "maximum_speed_m_s", positive=True))
        object.__setattr__(self, "body_sample_offset_m", _vector(self.body_sample_offset_m, "body_sample_offset_m"))


@dataclass(frozen=True, slots=True)
class ForcesSettings:
    air_density_kg_m3: float = DEFAULT_AIR_DENSITY_KG_M3
    rotor_radius_m: float = DEFAULT_ROTOR_RADIUS_M
    body_drag: BodyDragSettings = field(default_factory=BodyDragSettings)
    rotor_drag: RotorDragSettings = field(default_factory=RotorDragSettings)
    angular_damping: AngularDampingSettings = field(default_factory=AngularDampingSettings)
    inflow: InflowSettings = field(default_factory=InflowSettings)
    flapping: FlappingSettings = field(default_factory=FlappingSettings)
    gyroscopic: GyroscopicSettings = field(default_factory=GyroscopicSettings)
    ground_effect: GroundEffectSettings = field(default_factory=GroundEffectSettings)
    propwash: PropwashSettings = field(default_factory=PropwashSettings)

    def __post_init__(self) -> None:
        for name in ("air_density_kg_m3", "rotor_radius_m"):
            object.__setattr__(self, name, _number(getattr(self, name), name, positive=True))
        for name, settings_type in _force_setting_types().items():
            if not isinstance(getattr(self, name), settings_type):
                raise ConfigurationError(f"forces.{name} must be {settings_type.__name__}")

    @classmethod
    def ideal(cls) -> "ForcesSettings":
        """Explicit opt-out for analytical ideal-rotor experiments."""
        return cls(**{name: settings_type(enabled=False) for name, settings_type in _force_setting_types().items()})


@dataclass(frozen=True, slots=True)
class WindSettings:
    enabled: bool = True
    behavior: str = "constant"
    velocity_world_m_s: Vector3 = ZERO_VECTOR
    gust_stddev_m_s: Vector3 = ZERO_VECTOR
    correlation_time_s: float = DEFAULT_WIND_CORRELATION_TIME_S
    seed: int = DEFAULT_WIND_SEED

    def __post_init__(self) -> None:
        _sensor_options(self.enabled, self.seed)
        if not isinstance(self.behavior, str) or self.behavior not in ("constant", "gusts"):
            raise ConfigurationError("wind.behavior must be 'constant' or 'gusts'")
        object.__setattr__(self, "velocity_world_m_s", _vector(self.velocity_world_m_s, "wind.velocity_world_m_s"))
        values = _vector(self.gust_stddev_m_s, "wind.gust_stddev_m_s")
        if any(value < 0 for value in values):
            raise ConfigurationError("wind.gust_stddev_m_s must be nonnegative")
        object.__setattr__(self, "gust_stddev_m_s", values)
        object.__setattr__(self, "correlation_time_s", _number(self.correlation_time_s, "wind.correlation_time_s", positive=True))


@dataclass(frozen=True, slots=True)
class TelemetrySettings:
    """Select optional schemas; these switches do not start a recorder."""

    sensors: bool = True
    rc_input: bool = True
    control: bool = True
    allocation: bool = True
    forces: bool = True
    airflow: bool = True
    timing: bool = True

    def __post_init__(self) -> None:
        for setting in fields(self):
            if type(getattr(self, setting.name)) is not bool:
                raise ConfigurationError(f"telemetry.{setting.name} must be a boolean")


@dataclass(frozen=True, slots=True)
class PoseSources:
    altitude: str = 'ground_truth'
    attitude: str = 'ground_truth'

    def __post_init__(self) -> None:
        if self.altitude not in ('ground_truth', 'barometer'):
            raise ConfigurationError('pose_sources.altitude must be ground_truth or barometer')
        if self.attitude not in ('ground_truth', 'imu'):
            raise ConfigurationError('pose_sources.attitude must be ground_truth or imu')


@dataclass(frozen=True, slots=True)
class AttitudeEstimatorSettings:
    kp_s_inv: float = 2.0
    ki_s_inv2: float = 0.05
    acceleration_rejection_fraction: float = 0.2

    def __post_init__(self) -> None:
        for name in ('kp_s_inv', 'ki_s_inv2'):
            object.__setattr__(self, name, _nonnegative(getattr(self, name), f'attitude_estimator.{name}'))
        fraction = _number(self.acceleration_rejection_fraction, 'attitude_estimator.acceleration_rejection_fraction')
        if not 0 < fraction < 1:
            raise ConfigurationError('attitude_estimator.acceleration_rejection_fraction must be between 0 and 1')
        object.__setattr__(self, 'acceleration_rejection_fraction', fraction)


@dataclass(frozen=True, slots=True)
class SimulationConfig:
    simulation: SimulationSettings
    drone: DroneSettings
    control: ControlSettings = field(default_factory=ControlSettings)
    rc: RcSettings = field(default_factory=RcSettings)
    sensors: SensorsSettings = field(default_factory=SensorsSettings)
    forces: ForcesSettings = field(default_factory=ForcesSettings)
    wind: WindSettings = field(default_factory=WindSettings)
    telemetry: TelemetrySettings = field(default_factory=TelemetrySettings)
    pose_sources: PoseSources = field(default_factory=PoseSources)
    attitude_estimator: AttitudeEstimatorSettings = field(default_factory=AttitudeEstimatorSettings)

    def __post_init__(self) -> None:
        if not isinstance(self.pose_sources, PoseSources) or not isinstance(self.attitude_estimator, AttitudeEstimatorSettings):
            raise ConfigurationError('pose_sources/attitude_estimator settings have invalid types')
        if not isinstance(self.telemetry, TelemetrySettings):
            raise ConfigurationError('telemetry must be TelemetrySettings')
        if not isinstance(self.rc, RcSettings):
            raise ConfigurationError('rc must be RcSettings')
        if not isinstance(self.sensors, SensorsSettings):
            raise ConfigurationError('sensors must be SensorsSettings')
        if self.sensors.imu.enabled:
            self.sensors.imu.validate_sample_period(self.simulation.timestep_s)
        if not isinstance(self.forces, ForcesSettings) or not isinstance(self.wind, WindSettings):
            raise ConfigurationError('forces/wind must be ForcesSettings/WindSettings')
        if self.pose_sources.altitude == 'barometer' and not self.sensors.barometer.enabled:
            raise ConfigurationError('barometer pose source requires sensors.barometer.enabled')
        if self.pose_sources.attitude == 'imu':
            if not self.sensors.imu.enabled:
                raise ConfigurationError('imu pose source requires sensors.imu.enabled')
            gravity = hypot(*self.simulation.gravity_m_s2)
            if not isfinite(gravity) or gravity <= 0:
                raise ConfigurationError('imu attitude estimation requires finite nonzero gravity')


def _force_setting_types() -> dict:
    return {"body_drag": BodyDragSettings, "rotor_drag": RotorDragSettings,
            "angular_damping": AngularDampingSettings, "inflow": InflowSettings,
            "flapping": FlappingSettings, "gyroscopic": GyroscopicSettings,
            "ground_effect": GroundEffectSettings, "propwash": PropwashSettings}


def _mapping(value: object, name: str, required: set[str], optional: set[str] | None = None) -> dict:
    if not isinstance(value, dict):
        raise ConfigurationError(f"{name} must be a mapping")
    unknown = value.keys() - required - (optional or set())
    missing = required - value.keys()
    if unknown:
        raise ConfigurationError(f"{name} has unknown keys: {', '.join(sorted(map(str, unknown)))}")
    if missing:
        raise ConfigurationError(f"{name} is missing: {', '.join(sorted(missing))}")
    return value


class YamlConfigLoader:
    def load(self, path: str | Path) -> SimulationConfig:
        try:
            data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        except (OSError, UnicodeError, yaml.YAMLError) as exc:
            raise ConfigurationError(f"Cannot read YAML configuration {path}: {exc}") from exc
        root = _mapping(data, "configuration", {"simulation", "drone"}, {"control", "rc", "sensors", "forces", "wind", "telemetry", "pose_sources", "attitude_estimator"})
        simulation = _mapping(
            root["simulation"], "simulation",
            {"timestep_s", "max_steps", "gravity_m_s2"}, {"gui"},
        )
        drone = _mapping(root["drone"], "drone", {"mass_kg", "initial_position_m"}, {"drone_id"})
        control = ControlSettings(**_mapping(root["control"], "control", set(), {"backend"})) if "control" in root else ControlSettings()
        rc = RcSettings(**_mapping(root['rc'], 'rc', set(), set(RcSettings.__dataclass_fields__))) if 'rc' in root else RcSettings()
        sensors = _mapping(root.get("sensors", {}), "sensors", set(), {"imu", "barometer"})
        imu = ImuSettings(**_mapping(sensors.get("imu", {}), "sensors.imu", set(),
                                     set(ImuSettings.__dataclass_fields__)))
        barometer = BarometerSettings(**_mapping(sensors.get("barometer", {}), "sensors.barometer", set(),
                                                 set(BarometerSettings.__dataclass_fields__)))
        force_values = _mapping(root.get("forces", {}), "forces", set(), set(ForcesSettings.__dataclass_fields__))
        force_options = dict(force_values)
        for name, settings_type in _force_setting_types().items():
            force_options[name] = settings_type(**_mapping(force_values.get(name, {}), f"forces.{name}", set(),
                                                           set(settings_type.__dataclass_fields__)))
        forces = ForcesSettings(**force_options)
        wind = WindSettings(**_mapping(root.get("wind", {}), "wind", set(), set(WindSettings.__dataclass_fields__)))
        telemetry = TelemetrySettings(**_mapping(root.get("telemetry", {}), "telemetry", set(),
                                                 set(TelemetrySettings.__dataclass_fields__)))
        pose_sources = PoseSources(**_mapping(root.get('pose_sources', {}), 'pose_sources', set(),
                                               set(PoseSources.__dataclass_fields__)))
        estimator = AttitudeEstimatorSettings(**_mapping(root.get('attitude_estimator', {}), 'attitude_estimator', set(),
                                                         set(AttitudeEstimatorSettings.__dataclass_fields__)))
        return SimulationConfig(SimulationSettings(**simulation), DroneSettings(**drone), control, rc,
                                SensorsSettings(imu, barometer), forces, wind, telemetry, pose_sources, estimator)
