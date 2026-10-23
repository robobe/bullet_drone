"""Immutable data exchanged across simulation component boundaries."""

from dataclasses import dataclass, field
from math import isfinite, sqrt


DEFAULT_DRONE_ID = "drone_1"
MAX_DRONE_ID_LENGTH = 128
SURFACE_NORMAL_TOLERANCE = 1e-6


type Vector3 = tuple[float, float, float]
type Quaternion = tuple[float, float, float, float]
type RotorSpeeds = tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class DroneState:
    episode_id: str
    step_index: int
    simulation_time_s: float
    position_m: Vector3
    orientation_xyzw: Quaternion
    linear_velocity_m_s: Vector3
    angular_velocity_rad_s: Vector3
    rotor_speeds_rad_s: RotorSpeeds = (0.0, 0.0, 0.0, 0.0)
    drone_id: str = DEFAULT_DRONE_ID


@dataclass(frozen=True, slots=True)
class ImuReading:
    """Body-axis specific force and angular velocity at the center of mass."""

    episode_id: str
    step_index: int
    simulation_time_s: float
    specific_force_m_s2: Vector3
    angular_velocity_rad_s: Vector3


@dataclass(frozen=True, slots=True)
class BarometerReading:
    """Sensor output pressure (optionally filtered) and its world-Z altitude."""

    episode_id: str
    step_index: int
    simulation_time_s: float
    pressure_pa: float
    altitude_m: float


@dataclass(frozen=True, slots=True)
class SensorReadings:
    """Completed measurements; None means the sensor is not configured."""

    imu: ImuReading | None = None
    barometer: BarometerReading | None = None
    rangefinder: None = None


@dataclass(frozen=True, slots=True)
class AttitudeEstimate:
    """IMU-derived body-to-world XYZW attitude at the measurement time."""

    episode_id: str
    step_index: int
    simulation_time_s: float
    orientation_xyzw: Quaternion


@dataclass(frozen=True, slots=True)
class DroneObservation:
    ground_truth: DroneState
    sensors: SensorReadings = field(default_factory=SensorReadings)
    attitude_estimate: AttitudeEstimate | None = None


@dataclass(frozen=True, slots=True)
class ControlFeedback:
    """Ideal body-to-world orientation and body-axis angular rates (rad/s)."""

    orientation_xyzw: Quaternion
    body_rates_rad_s: Vector3


@dataclass(frozen=True, slots=True)
class PropellerOutput:
    """Body-frame rotor forces (N), and aggregate body reaction torque (N*m)."""

    forces_n: tuple[Vector3, Vector3, Vector3, Vector3]
    reaction_torque_nm: Vector3


@dataclass(frozen=True, slots=True)
class SurfaceHit:
    """Read-only surface below a world point: distance m, outward unit world normal."""

    distance_m: float
    normal_world: Vector3

    def __post_init__(self) -> None:
        if not isfinite(self.distance_m) or self.distance_m < 0:
            raise ValueError("Surface distance must be finite and nonnegative")
        if len(self.normal_world) != 3 or not all(isfinite(value) for value in self.normal_world):
            raise ValueError("Surface normal must have three finite components")
        length = sqrt(sum(value * value for value in self.normal_world))
        if not abs(length - 1) <= SURFACE_NORMAL_TOLERANCE:
            raise ValueError("Surface normal must be a unit vector")


@dataclass(frozen=True, slots=True)
class ForceOutput:
    """Four offset rotor forces N, COM force N and total body torque N*m; all body-frame."""

    rotor_forces_n: tuple[Vector3, Vector3, Vector3, Vector3]
    body_force_n: Vector3
    body_torque_nm: Vector3
