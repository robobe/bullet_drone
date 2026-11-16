"""Renderer collision proposal contracts."""
from typing import TypedDict

from .flight import _id, _integer, _number, _shape, _version, validate_source


COLLISION_TOPIC = b'render/drone_1/collision'
COLLISION_MAX_AGE_S = 1.0
COLLISION_RETRY_S = 0.1
TARGET_ID = 'red_truck'


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
