"""Request/reply simulation commands and legacy lifecycle publication validation."""
import json
import re
from dataclasses import dataclass, fields

from .wind import DictionaryMessage


COMMAND_MAX_AGE_S = 1.0
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


from .requests import CommandPayload, CommandRequest, CommandReply


def validate_command(value):
    CommandRequest.from_dict(value)
    return value
