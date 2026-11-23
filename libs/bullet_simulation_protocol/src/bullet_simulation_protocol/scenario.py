"""Renderer collision proposal contracts."""
from math import isfinite
from re import fullmatch
from typing import TypedDict

from .flight import _id, _integer, _number, _shape, _version, validate_source


COLLISION_TOPIC = b'render/drone_1/collision'
CONTACT_TOPIC = b'render/drone_1/contact'
COLLISION_MAX_AGE_S = 1.0
COLLISION_RETRY_S = 0.1
TARGET_ID = 'red_truck'


def validate_contact(value):
    """Validate renderer-confirmed truck contact for minimal-pose streams."""
    required = {'schema_version', 'event_id', 'stream_id', 'step_index', 'target_id', 'drone_position_m'}
    if (not isinstance(value, dict) or set(value) != required
            or type(value['schema_version']) is not int or value['schema_version'] != 1):
        raise ValueError('invalid contact fields or version')
    for key in ('event_id', 'stream_id'):
        if not isinstance(value[key], str) or not fullmatch(r'[A-Za-z0-9_-]{1,128}', value[key]):
            raise ValueError(f'invalid contact {key}')
    if type(value['step_index']) is not int or not 0 <= value['step_index'] <= 2**53 - 1:
        raise ValueError('invalid contact step_index')
    if value['target_id'] != TARGET_ID:
        raise ValueError('invalid contact target')
    position = value['drone_position_m']
    if (not isinstance(position, (list, tuple)) or len(position) != 3
            or any(type(component) not in (int, float) or abs(component) > 1e12 or not isfinite(component)
                   for component in position)):
        raise ValueError('invalid contact position')
    return value


class CollisionReport(TypedDict):
    schema_version: int
    event_id: str
    controller_id: str
    issued_us: int
    source: dict
    target_id: str
    drone_position_m: list[float]


def validate_collision(value):
    _shape(value, set(CollisionReport.__annotations__))
    _version(value['schema_version'])
    _id(value['event_id'])
    _id(value['controller_id'])
    _integer(value['issued_us'])
    validate_source(value['source'])
    if value['target_id'] != TARGET_ID or value['issued_us'] < value['source']['measurement_us']:
        raise ValueError('invalid collision target/time')
    p = value['drone_position_m']
    if not isinstance(p, (list, tuple)) or len(p) != 3:
        raise ValueError('invalid collision position')
    for component in p:
        _number(component)
    return value
