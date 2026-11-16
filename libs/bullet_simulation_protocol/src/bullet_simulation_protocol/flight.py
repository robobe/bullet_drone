"""Local expiring RC intent, sensor feedback and capture-source contracts."""
import json
import math

from .control import MAX_SEQUENCE, identity


FEEDBACK_TOPIC = b'flight/drone_1/feedback'
MISSION_TOPIC = b'app/drone_1/mission'
MAX_FLIGHT_BYTES = 8192
SOURCE_FIELDS = {'physics_id', 'episode_id', 'stream_id', 'step_index', 'simulation_time_s', 'measurement_us'}
FEEDBACK_FIELDS = {'schema_version', 'source', 'barometer_altitude_m', 'orientation_xyzw', 'armed', 'mode',
                   'last_applied_sequence', 'owner_id', 'state', 'reason'}
RANGEFINDER_FIELDS = {'distance_cm', 'valid'}


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
    version = value.get('schema_version') if isinstance(value, dict) else None
    allowed = FEEDBACK_FIELDS | ({'takeoff_height_m'} if version == 2 else set())
    if isinstance(value, dict) and 'rangefinder' in value:
        allowed.add('rangefinder')
    _shape(value, allowed)
    _integer(version, 1, 2)
    if version == 2:
        from .spawn import validate_takeoff_height
        validate_takeoff_height(value['takeoff_height_m'])
    validate_source(value['source'])
    if value['barometer_altitude_m'] is not None:
        _number(value['barometer_altitude_m'])
    if 'rangefinder' in value and value['rangefinder'] is not None:
        rangefinder = value['rangefinder']
        _shape(rangefinder, RANGEFINDER_FIELDS)
        if type(rangefinder['valid']) is not bool:
            raise ValueError('invalid rangefinder validity')
        if rangefinder['distance_cm'] is not None:
            _number(rangefinder['distance_cm'])
            if rangefinder['distance_cm'] < 0:
                raise ValueError('negative rangefinder distance')
    q = value['orientation_xyzw']
    if not isinstance(q, (list, tuple)) or len(q) != 4:
        raise ValueError('invalid attitude')
    for component in q:
        _number(component)
    if abs(sum(v * v for v in q) - 1) > 0.002:
        raise ValueError('attitude is not a unit quaternion')
    if type(value['armed']) is not bool or value['mode'] not in ('angle', 'acro'):
        raise ValueError('invalid applied arm/mode')
    _integer(value['last_applied_sequence'])
    if value['owner_id'] is not None:
        _id(value['owner_id'])
    if value['state'] not in ('stopped', 'running', 'error') or not isinstance(value['reason'], str) or not 0 < len(value['reason']) <= 256:
        raise ValueError('invalid flight state')
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
