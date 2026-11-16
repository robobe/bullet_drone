"""Spawn mutation contracts and retained numeric bounds."""
from .flight import _number
from .requests import SpawnRequest, SpawnReply, SpawnPayload

HORIZONTAL_LIMIT_M = 250.
MIN_ALTITUDE_M = 0.015
MAX_ALTITUDE_M = 500.
DEFAULT_TAKEOFF_HEIGHT_M = 20.
MIN_TAKEOFF_HEIGHT_M = 0.1
MAX_TAKEOFF_HEIGHT_M = 100.

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
