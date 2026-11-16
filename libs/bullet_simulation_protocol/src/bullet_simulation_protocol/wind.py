"""Process-neutral wind request/reply dataclasses, without JSON Schema."""
from dataclasses import dataclass, asdict, fields
from math import isfinite

MAX_WIRE_INTEGER = 2**53 - 1
MAX_WIND_VALUE = 1e12


def shape(value, cls):
    if not isinstance(value, dict) or set(value) != {f.name for f in fields(cls)}:
        raise ValueError("invalid wind message fields")


def number(value, nonnegative=False):
    if type(value) not in (int, float) or not isfinite(value) or abs(value) > MAX_WIND_VALUE or (nonnegative and value < 0):
        raise ValueError("invalid wind number")


def identity(value):
    if not isinstance(value, str) or not value or len(value) > 128:
        raise ValueError("invalid wind identity")


class DictionaryMessage:
    def to_dict(self):
        def plain(value):
            if isinstance(value, dict):
                return {key: plain(item) for key, item in value.items()}
            if isinstance(value, (tuple, list)):
                return [plain(item) for item in value]
            return value
        return plain(asdict(self))


@dataclass(frozen=True)
class WindSettingsMessage(DictionaryMessage):
    enabled: bool
    behavior: str
    velocity_world_m_s: tuple[float, float, float]
    gust_stddev_m_s: tuple[float, float, float]
    correlation_time_s: float
    seed: int

    def __post_init__(self):
        if type(self.enabled) is not bool or self.behavior not in ('constant', 'gusts'):
            raise ValueError("invalid wind behavior")
        for vector in (self.velocity_world_m_s, self.gust_stddev_m_s):
            if not isinstance(vector, tuple) or len(vector) != 3:
                raise ValueError("invalid wind vector")
        for value in self.velocity_world_m_s:
            number(value)
        for value in self.gust_stddev_m_s:
            number(value, True)
        number(self.correlation_time_s)
        if self.correlation_time_s <= 0 or type(self.seed) is not int or abs(self.seed) > MAX_WIRE_INTEGER:
            raise ValueError("invalid wind seed or correlation")

    @classmethod
    def from_dict(cls, value):
        shape(value, cls)
        try:
            return cls(**dict(value, velocity_world_m_s=tuple(value['velocity_world_m_s']),
                              gust_stddev_m_s=tuple(value['gust_stddev_m_s'])))
        except TypeError as error:
            raise ValueError("invalid wind settings") from error


@dataclass(frozen=True)
class GetWindRequest(DictionaryMessage):
    action: str = 'get_wind'

    def __post_init__(self):
        if self.action != 'get_wind':
            raise ValueError("invalid wind action")

    @classmethod
    def from_dict(cls, value):
        shape(value, cls)
        return cls(**value)


@dataclass(frozen=True)
class SetWindRequest(DictionaryMessage):
    physics_id: str
    episode_id: str
    issued_ms: int
    settings: WindSettingsMessage
    action: str = 'set_wind'

    def __post_init__(self):
        identity(self.physics_id)
        identity(self.episode_id)
        if self.action != 'set_wind' or type(self.issued_ms) is not int or not 0 <= self.issued_ms <= MAX_WIRE_INTEGER or not isinstance(self.settings, WindSettingsMessage):
            raise ValueError("invalid wind request")

    @classmethod
    def from_dict(cls, value):
        shape(value, cls)
        return cls(**dict(value, settings=WindSettingsMessage.from_dict(value['settings'])))


@dataclass(frozen=True)
class WindReadback(DictionaryMessage):
    physics_id: str
    episode_id: str
    step_index: int
    simulation_time_s: float
    settings: WindSettingsMessage
    velocity_world_m_s: tuple[float, float, float]

    def __post_init__(self):
        identity(self.physics_id)
        identity(self.episode_id)
        if type(self.step_index) is not int or not 0 <= self.step_index <= MAX_WIRE_INTEGER or not isinstance(self.settings, WindSettingsMessage):
            raise ValueError("invalid wind snapshot")
        number(self.simulation_time_s, True)
        if not isinstance(self.velocity_world_m_s, tuple) or len(self.velocity_world_m_s) != 3:
            raise ValueError("invalid actual wind")
        for value in self.velocity_world_m_s:
            number(value)

    @classmethod
    def from_dict(cls, value):
        shape(value, cls)
        return cls(**dict(value, settings=WindSettingsMessage.from_dict(value['settings']),
                          velocity_world_m_s=tuple(value['velocity_world_m_s'])))


@dataclass(frozen=True)
class WindReply(DictionaryMessage):
    status: str
    reason: str
    state: WindReadback

    def __post_init__(self):
        if self.status not in ('current', 'applied', 'rejected') or not isinstance(self.reason, str) or not 0 < len(self.reason) <= 256 or not isinstance(self.state, WindReadback):
            raise ValueError("invalid wind reply")

    @classmethod
    def from_dict(cls, value):
        shape(value, cls)
        return cls(**dict(value, state=WindReadback.from_dict(value['state'])))
