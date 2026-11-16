"""Mutation-only request/reply contracts. Topics are application routing fields."""
from dataclasses import dataclass

from .wind import DictionaryMessage, WindSettingsMessage

POSITION_XY_LIMIT_M = 250.
MIN_POSITION_Z_M = 0.015
MAX_POSITION_Z_M = 500.
MAX_DRONE_ID_LENGTH = 128
MAX_REASON_LENGTH = 256


def shape(value, fields):
    if not isinstance(value, dict) or set(value) != fields:
        raise ValueError('invalid_request_fields')


@dataclass(frozen=True)
class SpawnPayload(DictionaryMessage):
    position_m: tuple[float, float, float]

    def __post_init__(self):
        from math import isfinite
        values = self.position_m
        if not isinstance(values, tuple) or len(values) != 3 or any(type(x) not in (int, float) or abs(x) > MAX_POSITION_Z_M or not isfinite(x) for x in values):
            raise ValueError('invalid_spawn_position')
        if any(abs(x) > POSITION_XY_LIMIT_M for x in values[:2]) or not MIN_POSITION_Z_M <= values[2] <= MAX_POSITION_Z_M:
            raise ValueError('spawn_position_out_of_bounds')

    @classmethod
    def from_dict(cls, value):
        shape(value, {'position_m'})
        if not isinstance(value['position_m'], (list, tuple)):
            raise ValueError('invalid_spawn_position')
        return cls(tuple(value['position_m']))


@dataclass(frozen=True)
class CommandPayload(DictionaryMessage):
    action: str

    def __post_init__(self):
        if self.action not in ('start', 'stop', 'reset'):
            raise ValueError('invalid_command')

    @classmethod
    def from_dict(cls, value):
        shape(value, {'action'})
        return cls(**value)


class Request(DictionaryMessage):
    def __post_init__(self):
        if not isinstance(self.drone_id, str) or not self.drone_id.strip() or len(self.drone_id) > MAX_DRONE_ID_LENGTH:
            raise ValueError('invalid_drone_id')
        if self.topic != self._TOPIC or not isinstance(self.payload, self._PAYLOAD):
            raise ValueError('invalid_topic_payload')

    @classmethod
    def from_dict(cls, value):
        shape(value, {'topic', 'drone_id', 'payload'})
        return cls(value['drone_id'], cls._PAYLOAD.from_dict(value['payload']), value['topic'])


@dataclass(frozen=True)
class SpawnRequest(Request):
    drone_id: str
    payload: SpawnPayload
    topic: str = 'spawn'
    _TOPIC = 'spawn'
    _PAYLOAD = SpawnPayload


@dataclass(frozen=True)
class WindRequest(Request):
    drone_id: str
    payload: WindSettingsMessage
    topic: str = 'wind'
    _TOPIC = 'wind'
    _PAYLOAD = WindSettingsMessage


@dataclass(frozen=True)
class CommandRequest(Request):
    drone_id: str
    payload: CommandPayload
    topic: str = 'command'
    _TOPIC = 'command'
    _PAYLOAD = CommandPayload


@dataclass(frozen=True)
class OperationReply(DictionaryMessage):
    success: bool
    reason: str | None = None

    def __post_init__(self):
        if type(self.success) is not bool:
            raise ValueError('invalid_reply_success')
        if self.success:
            if self.reason is not None:
                raise ValueError('successful_reply_has_no_reason')
        elif not isinstance(self.reason, str) or not self.reason.strip() or len(self.reason) > MAX_REASON_LENGTH:
            raise ValueError('failure_requires_reason')

    def to_dict(self):
        return {'success': True} if self.success else {'success': False, 'reason': self.reason}

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict):
            raise ValueError('invalid_reply')
        shape(value, {'success'} if value.get('success') is True else {'success', 'reason'})
        return cls(value['success'], value.get('reason'))


SpawnReply = OperationReply
WindReply = OperationReply
CommandReply = OperationReply



def parse_request(value):
    if not isinstance(value, dict):
        raise ValueError('unknown_request_topic')
    if value.get('topic') == 'spawn':
        return SpawnRequest.from_dict(value)
    if value.get('topic') == 'wind':
        return WindRequest.from_dict(value)
    if value.get('topic') == 'command':
        return CommandRequest.from_dict(value)
    raise ValueError('unknown_request_topic')
