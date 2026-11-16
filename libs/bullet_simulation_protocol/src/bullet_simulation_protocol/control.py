"""Request/reply simulation commands and legacy lifecycle publication validation."""
import json
import re
from dataclasses import dataclass, fields

from .wind import DictionaryMessage


STATE_TOPIC = b'simulation/state'
HEARTBEAT_INTERVAL_S = 0.5
HEARTBEAT_TIMEOUT_S = 3.0
COMMAND_MAX_AGE_S = 1.0
STATE_INTERVAL_S = 1.0
MAX_CONTROL_BYTES = 4096
MAX_SEQUENCE = 2**53 - 1
ACTIONS = ('start', 'stop', 'reset')


def identity(value):
    return isinstance(value, str) and re.fullmatch(r'[A-Za-z0-9_-]{1,128}', value) is not None


def decode_json(payload):
    if not 0 < len(payload) <= MAX_CONTROL_BYTES:
        raise ValueError('control payload exceeds size limit')
    def invalid(value):
        raise ValueError(f'invalid JSON constant {value}')
    return json.loads(payload.decode('utf-8'), parse_constant=invalid)


class CommandMessage(DictionaryMessage):
    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict) or set(value) != {f.name for f in fields(cls)}:
            raise ValueError('invalid command message fields')
        return cls(**value)


@dataclass(frozen=True)
class SimulationCommand(CommandMessage):
    action: str
    physics_id: str
    episode_id: str
    issued_ms: int

    def __post_init__(self):
        if self.action not in ACTIONS or not identity(self.physics_id) or not identity(self.episode_id):
            raise ValueError('invalid command action or simulation identity')
        if type(self.issued_ms) is not int or not 0 <= self.issued_ms <= MAX_SEQUENCE:
            raise ValueError('invalid command issue time')


@dataclass(frozen=True)
class GetSimulationState(CommandMessage):
    action: str = 'get_simulation_state'

    def __post_init__(self):
        if self.action != 'get_simulation_state':
            raise ValueError('invalid state query')


@dataclass(frozen=True)
class SimulationStatus(CommandMessage):
    physics_id: str
    episode_id: str
    state: str
    reason: str
    simulation_time_s: float
    ready_to_start: bool

    def __post_init__(self):
        if not identity(self.physics_id) or not identity(self.episode_id) or self.state not in ('running', 'stopped', 'error'):
            raise ValueError('invalid simulation status')
        if not isinstance(self.reason, str) or not 0 < len(self.reason) <= 256:
            raise ValueError('invalid status reason')
        if type(self.simulation_time_s) not in (int, float) or not 0 <= self.simulation_time_s <= 1e12:
            raise ValueError('invalid status time')
        if type(self.ready_to_start) is not bool or self.ready_to_start and self.state != 'stopped':
            raise ValueError('invalid start readiness')


@dataclass(frozen=True)
class CommandReply(CommandMessage):
    status: str
    reason: str
    state: SimulationStatus

    def __post_init__(self):
        if self.status not in ('applied', 'current', 'rejected') or not isinstance(self.state, SimulationStatus):
            raise ValueError('invalid command reply')
        if not isinstance(self.reason, str) or not 0 < len(self.reason) <= 256:
            raise ValueError('invalid command reason')

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict) or set(value) != {'status', 'reason', 'state'}:
            raise ValueError('invalid command reply fields')
        return cls(value['status'], value['reason'], SimulationStatus.from_dict(value['state']))


def validate_command(value):
    SimulationCommand.from_dict(value)
    return value


def validate_state(value):
    fields = {'schema_version', 'physics_id', 'controller_id', 'episode_id', 'state', 'reason', 'simulation_time_s'}
    if not isinstance(value, dict) or set(value) != fields:
        raise ValueError('invalid simulation state fields')
    if type(value['schema_version']) is not int or value['schema_version'] != 1 or not identity(value['physics_id']):
        raise ValueError('invalid state version or physics identity')
    if value['controller_id'] is not None and not identity(value['controller_id']):
        raise ValueError('invalid state controller identity')
    if not identity(value['episode_id']) or value['state'] not in ('running', 'stopped', 'error'):
        raise ValueError('invalid episode or state')
    if not isinstance(value['reason'], str) or not 0 < len(value['reason']) <= 256:
        raise ValueError('invalid state reason')
    elapsed = value['simulation_time_s']
    if type(elapsed) not in (int, float) or not 0 <= elapsed <= 1e12:
        raise ValueError('invalid simulation time')
    return value

