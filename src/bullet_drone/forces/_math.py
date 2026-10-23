"""Small vector operations internal to aerodynamic calculations."""

from math import sqrt

from bullet_drone.models import Vector3


ZERO_VECTOR = (0.0, 0.0, 0.0)
BODY_UP = (0.0, 0.0, 1.0)


def add(a: Vector3, b: Vector3) -> Vector3:
    return tuple(x + y for x, y in zip(a, b))


def scale(a: Vector3, factor: float) -> Vector3:
    return tuple(x * factor for x in a)


def dot(a: Vector3, b: Vector3) -> float:
    return sum(x * y for x, y in zip(a, b))


def norm(a: Vector3) -> float:
    return sqrt(dot(a, a))


def cross(a: Vector3, b: Vector3) -> Vector3:
    x, y, z = a
    u, v, w = b
    return (y*w - z*v, z*u - x*w, x*v - y*u)
