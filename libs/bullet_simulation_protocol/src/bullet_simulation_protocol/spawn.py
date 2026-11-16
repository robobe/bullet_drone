"""Stopped-episode spawn request/reply dataclasses in Bullet metres (Z up)."""
from dataclasses import dataclass

from .control import identity
from .flight import _shape, _integer, _number

from .wind import DictionaryMessage

SPAWN_ACTION = 'set_spawn'
HORIZONTAL_LIMIT_M = 250.0
MIN_ALTITUDE_M = 0.015
MAX_ALTITUDE_M = 500.0
DEFAULT_TAKEOFF_HEIGHT_M = 20.0
MIN_TAKEOFF_HEIGHT_M = 0.1
MAX_TAKEOFF_HEIGHT_M = 100.0


@dataclass(frozen=True)
class SpawnRequest(DictionaryMessage):
    schema_version: int
    controller_id: str
    physics_id: str
    episode_id: str
    request_id: str
    issued_ms: int
    scenario_name: str
    position_m: tuple[float, float, float]
    takeoff_height_m: float
    action: str = SPAWN_ACTION

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict) or set(value) != set(cls.__dataclass_fields__):
            raise ValueError('invalid spawn request fields')
        try:
            return cls(**dict(value, position_m=tuple(value['position_m'])))
        except (TypeError, ValueError) as error:
            raise ValueError('invalid spawn request') from error

    def __post_init__(self):
        _integer(self.schema_version, 2, 2)
        if self.action != SPAWN_ACTION:
            raise ValueError('invalid spawn action')
        for key in ('controller_id', 'physics_id', 'episode_id', 'request_id'):
            if not identity(getattr(self, key)):
                raise ValueError('invalid spawn identity')
        _integer(self.issued_ms)
        _name(self.scenario_name)
        if not isinstance(self.position_m, tuple):
            raise ValueError('expected immutable XYZ tuple')
        validate_position(self.position_m)
        validate_takeoff_height(self.takeoff_height_m)


@dataclass(frozen=True)
class SpawnState(DictionaryMessage):
    schema_version: int
    physics_id: str
    controller_id: str | None
    episode_id: str
    scenario_name: str
    position_m: tuple[float, float, float]
    takeoff_height_m: float
    request_id: str | None
    result: str
    reason: str

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict) or set(value) != set(cls.__dataclass_fields__):
            raise ValueError('invalid spawn state fields')
        try:
            return cls(**dict(value, position_m=tuple(value['position_m'])))
        except (TypeError, ValueError) as error:
            raise ValueError('invalid spawn state') from error

    def __post_init__(self):
        _integer(self.schema_version, 2, 2)
        for key in ('physics_id', 'episode_id'):
            if not identity(getattr(self, key)):
                raise ValueError('invalid spawn identity')
        for key in ('controller_id', 'request_id'):
            value = getattr(self, key)
            if value is not None and not identity(value):
                raise ValueError('invalid optional spawn identity')
        _name(self.scenario_name)
        if not isinstance(self.position_m, tuple):
            raise ValueError('expected immutable XYZ tuple')
        validate_position(self.position_m)
        validate_takeoff_height(self.takeoff_height_m)
        if self.result not in ('idle', 'applied', 'rejected') or not isinstance(self.reason, str) or not 0 < len(self.reason) <= 256:
            raise ValueError('invalid spawn outcome')
        if (self.request_id is None) != (self.result == 'idle'):
            raise ValueError('spawn outcome requires request identity')


def validate_position(position):
    if not isinstance(position, (list, tuple)) or len(position) != 3:
        raise ValueError('expected XYZ position')
    for n in position:
        _number(n)
    if any(abs(n) > HORIZONTAL_LIMIT_M for n in position[:2]) or not MIN_ALTITUDE_M <= position[2] <= MAX_ALTITUDE_M:
        raise ValueError('spawn outside landscape/altitude bounds')
    return position


def _name(name):
    if not isinstance(name, str) or not 0 < len(name) <= 64 or name != name.strip() or any(ord(c) < 32 for c in name):
        raise ValueError('invalid scenario name')


def validate_takeoff_height(value):
    _number(value)
    if not MIN_TAKEOFF_HEIGHT_M <= value <= MAX_TAKEOFF_HEIGHT_M:
        raise ValueError('takeoff height must be 0.1–100 m')
    return value


def validate_request(value):
    SpawnRequest.from_dict(value)
    return value


def validate_state(value):
    SpawnState.from_dict(value)
    return value


@dataclass(frozen=True)
class GetSpawnRequest(DictionaryMessage):
    action: str = 'get_spawn'

    def __post_init__(self):
        if self.action != 'get_spawn':
            raise ValueError('invalid spawn action')

    @classmethod
    def from_dict(cls, value):
        _shape(value, {'action'})
        return cls(**value)


@dataclass(frozen=True)
class SpawnReply(DictionaryMessage):
    status: str
    reason: str
    state: SpawnState

    def __post_init__(self):
        if self.status not in ('current', 'applied', 'rejected'):
            raise ValueError('invalid spawn reply status')
        if not isinstance(self.reason, str) or not 0 < len(self.reason) <= 256 or not isinstance(self.state, SpawnState):
            raise ValueError('invalid spawn reply')

    @classmethod
    def from_dict(cls, value):
        _shape(value, {'status', 'reason', 'state'})
        return cls(value['status'], value['reason'], SpawnState.from_dict(value['state']))
