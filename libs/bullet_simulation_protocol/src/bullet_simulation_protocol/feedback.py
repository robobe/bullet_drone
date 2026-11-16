"""Complete observation telemetry, with simulation time and explicit frames."""
from dataclasses import dataclass, fields
from math import isfinite

from .wind import DictionaryMessage

NANOSECONDS_PER_SECOND = 1_000_000_000
MAX_SECONDS = 10**12
QUATERNION_NORM_TOLERANCE = 0.002
VECTOR_FIELDS = {'position_m': 3, 'orientation_xyzw': 4, 'linear_velocity_m_s': 3,
                 'angular_velocity_rad_s': 3, 'rotor_speeds_rad_s': 4,
                 'specific_force_m_s2': 3, 'velocity_enu_m_s': 3}


def number(value):
    if type(value) not in (int, float) or abs(value) > MAX_SECONDS or not isfinite(value):
        raise ValueError('expected finite telemetry number')


class Message(DictionaryMessage):
    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict) or set(value) != {f.name for f in fields(cls)}:
            raise ValueError(f'invalid {cls.__name__} fields')
        data = dict(value)
        for key, nested in NESTED.get(cls, {}).items():
            if data[key] is not None:
                data[key] = nested.from_dict(data[key])
        for key in VECTOR_FIELDS.keys() & data.keys():
            if not isinstance(data[key], (list, tuple)):
                raise ValueError('expected telemetry vector')
            data[key] = tuple(data[key])
        return cls(**data)

    def __post_init__(self):
        for key, nested in NESTED.get(type(self), {}).items():
            value = getattr(self, key)
            if value is None and (type(self) is Sensors or key == 'attitude_estimate'):
                continue
            if not isinstance(value, nested):
                raise ValueError(f'invalid {key}')
        for field in fields(self):
            key, value = field.name, getattr(self, field.name)
            if key in VECTOR_FIELDS:
                if not isinstance(value, tuple) or len(value) != VECTOR_FIELDS[key]:
                    raise ValueError(f'invalid {key}')
                for component in value:
                    number(component)
                if key == 'orientation_xyzw' and abs(sum(x*x for x in value)-1) > QUATERNION_NORM_TOLERANCE:
                    raise ValueError('expected unit quaternion')
                if key == 'rotor_speeds_rad_s' and any(x < 0 for x in value):
                    raise ValueError('negative rotor speed')
            elif key not in NESTED.get(type(self), {}):
                if key == 'valid':
                    if type(value) is not bool:
                        raise ValueError('expected rangefinder validity')
                elif value is None and key == 'distance_cm':
                    continue
                else:
                    number(value)


@dataclass(frozen=True)
class TimeStamp(Message):
    sec: int
    nanosec: int

    def __post_init__(self):
        if type(self.sec) is not int or not 0 <= self.sec <= MAX_SECONDS or type(self.nanosec) is not int or not 0 <= self.nanosec < NANOSECONDS_PER_SECOND:
            raise ValueError('invalid simulation timestamp')

    @classmethod
    def from_seconds(cls, seconds):
        number(seconds)
        if seconds < 0:
            raise ValueError('negative simulation time')
        total = round(seconds * NANOSECONDS_PER_SECOND)
        return cls(*divmod(total, NANOSECONDS_PER_SECOND))


@dataclass(frozen=True)
class Header(Message):
    frame_id: str
    time_stamp: TimeStamp

    def __post_init__(self):
        if not isinstance(self.frame_id, str) or not self.frame_id.strip() or len(self.frame_id) > 256 or not isinstance(self.time_stamp, TimeStamp):
            raise ValueError('invalid telemetry header')


@dataclass(frozen=True)
class GroundTruth(Message):
    header: Header
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    altitude_m: float
    linear_velocity_m_s: tuple[float, float, float]
    angular_velocity_rad_s: tuple[float, float, float]
    rotor_speeds_rad_s: tuple[float, float, float, float]

    def __post_init__(self):
        super().__post_init__()
        if self.altitude_m != self.position_m[2]:
            raise ValueError('ground truth altitude must equal world Z')


@dataclass(frozen=True)
class Imu(Message):
    header: Header
    specific_force_m_s2: tuple[float, float, float]
    angular_velocity_rad_s: tuple[float, float, float]


@dataclass(frozen=True)
class Barometer(Message):
    header: Header
    pressure_pa: float
    altitude_m: float

    def __post_init__(self):
        super().__post_init__()
        if self.pressure_pa <= 0:
            raise ValueError('pressure must be positive')


@dataclass(frozen=True)
class Rangefinder(Message):
    header: Header
    distance_cm: float | None
    valid: bool

    def __post_init__(self):
        super().__post_init__()
        if self.distance_cm is not None and self.distance_cm < 0 or self.valid != (self.distance_cm is not None):
            raise ValueError('invalid rangefinder return')


@dataclass(frozen=True)
class Gps(Message):
    header: Header
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    velocity_enu_m_s: tuple[float, float, float]

    def __post_init__(self):
        super().__post_init__()
        if not -90 <= self.latitude_deg <= 90 or not -180 <= self.longitude_deg <= 180:
            raise ValueError('invalid geographic position')


@dataclass(frozen=True)
class AttitudeEstimate(Message):
    header: Header
    orientation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class Sensors(Message):
    imu: Imu | None
    barometer: Barometer | None
    rangefinder: Rangefinder | None
    gps: Gps | None


@dataclass(frozen=True)
class FlightFeedback(Message):
    header: Header
    drone_id: str
    ground_truth: GroundTruth
    sensors: Sensors
    attitude_estimate: AttitudeEstimate | None

    def __post_init__(self):
        if not isinstance(self.drone_id, str) or not self.drone_id.strip() or len(self.drone_id) > 128:
            raise ValueError('invalid drone ID')
        # Identity is textual, so validate the numeric/nested fields separately.
        for key, nested in NESTED[type(self)].items():
            value = getattr(self, key)
            if key == 'attitude_estimate' and value is None:
                continue
            if not isinstance(value, nested):
                raise ValueError(f'invalid {key}')


NESTED = {Header: {'time_stamp': TimeStamp},
          GroundTruth: {'header': Header}, Imu: {'header': Header},
          Barometer: {'header': Header}, Rangefinder: {'header': Header},
          Gps: {'header': Header}, AttitudeEstimate: {'header': Header},
          Sensors: {'imu': Imu, 'barometer': Barometer, 'rangefinder': Rangefinder, 'gps': Gps},
          FlightFeedback: {'header': Header, 'ground_truth': GroundTruth,
                           'sensors': Sensors, 'attitude_estimate': AttitudeEstimate}}
