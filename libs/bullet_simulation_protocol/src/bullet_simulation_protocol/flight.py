"""Capture-source helpers and complete simulation feedback validation."""
import json
import math

from .control import MAX_SEQUENCE, identity


FEEDBACK_TOPIC = b'flight/drone_1/feedback'
MISSION_TOPIC = b'app/drone_1/mission'
MAX_FLIGHT_BYTES = 8192
SOURCE_FIELDS = {'physics_id', 'episode_id', 'stream_id', 'step_index', 'simulation_time_s', 'measurement_us'}


def _shape(value, fields):
    if not isinstance(value, dict) or set(value) != fields:
        raise ValueError('invalid flight message fields')


def _integer(value, lower=0, upper=MAX_SEQUENCE):
    if type(value) is not int or not lower <= value <= upper:
        raise ValueError('invalid flight integer')


def _number(value):
    if type(value) not in (float, int) or abs(value) > 1e12 or not math.isfinite(value):
        raise ValueError('invalid flight number')


def _id(value):
    if not identity(value):
        raise ValueError('invalid flight identity')


def _version(value):
    _integer(value, 1, 1)


def validate_source(value):
    _shape(value, SOURCE_FIELDS)
    for name in ('physics_id', 'episode_id', 'stream_id'):
        _id(value[name])
    for name in ('step_index', 'measurement_us'):
        _integer(value[name])
    _number(value['simulation_time_s'])
    if value['simulation_time_s'] < 0:
        raise ValueError('negative simulation time')
    return value


def validate_feedback(value):
    from .feedback import FlightFeedback
    FlightFeedback.from_dict(value)
    return value


def decode(payload, validator):
    if not 0 < len(payload) <= MAX_FLIGHT_BYTES:
        raise ValueError('flight payload exceeds bounds')
    def reject(value):
        raise ValueError(f'invalid JSON constant: {value}')
    try:
        value = json.loads(payload.decode('utf8'), parse_constant=reject)
    except (RecursionError, OverflowError) as exc:
        raise ValueError('flight JSON exceeds parser bounds') from exc
    return validator(value)


def encode(value, validator):
    validator(value)
    raw = json.dumps(value, separators=(',', ':'), allow_nan=False).encode()
    if len(raw) > MAX_FLIGHT_BYTES:
        raise ValueError('flight payload exceeds bounds')
    return raw
