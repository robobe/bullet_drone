"""Right-handed quaternion math: xyzw, body X forward/Y left/Z up."""

from math import atan2, cos, isfinite, sin, sqrt

from bullet_drone.models import Quaternion, Vector3


VECTOR_COMPONENTS = 3
QUATERNION_COMPONENTS = 4
SMALL_ROTATION_VECTOR_NORM = 1e-12


class GeometryError(ValueError):
    """A vector or quaternion cannot represent valid frame math input."""


def _values(value: object, count: int, name: str) -> tuple[float, ...]:
    if not isinstance(value, (tuple, list)) or len(value) != count:
        raise GeometryError(f"{name} must contain exactly {count} numbers")
    result = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise GeometryError(f"{name} must contain finite numbers")
        try:
            number = float(item)
        except OverflowError as exc:
            raise GeometryError(f"{name} must contain finite numbers") from exc
        if not isfinite(number):
            raise GeometryError(f"{name} must contain finite numbers")
        result.append(number)
    return tuple(result)


def normalize(q: Quaternion) -> Quaternion:
    q = _values(q, QUATERNION_COMPONENTS, "orientation_xyzw")
    norm = sqrt(sum(v * v for v in q))
    if not 0 < norm < float("inf"):
        raise GeometryError("Orientation must have finite, nonzero norm")
    return tuple(v / norm for v in q)


def conjugate(q: Quaternion) -> Quaternion:
    q = _values(q, QUATERNION_COMPONENTS, "quaternion")
    return (-q[0], -q[1], -q[2], q[3])


def multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    x, y, z, w = _values(a, QUATERNION_COMPONENTS, "left_quaternion")
    u, v, t, s = _values(b, QUATERNION_COMPONENTS, "right_quaternion")
    return (w*u + x*s + y*t - z*v, w*v - x*t + y*s + z*u,
            w*t + x*v - y*u + z*s, w*s - x*u - y*v - z*t)


def from_rpy(rpy: Vector3) -> Quaternion:
    rpy = _values(rpy, VECTOR_COMPONENTS, "rpy_rad")
    r, p, y = (v / 2 for v in rpy)
    cr, sr, cp, sp, cy, sy = cos(r), sin(r), cos(p), sin(p), cos(y), sin(y)
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)


def world_to_body(vector: Vector3, orientation: Quaternion) -> Vector3:
    vector = _values(vector, VECTOR_COMPONENTS, "vector")
    q = normalize(orientation)
    return multiply(multiply(conjugate(q), (*vector, 0.0)), q)[:3]


def attitude_error(current: Quaternion, target: Quaternion) -> Vector3:
    """Shortest rotation vector in current body coordinates, radians."""
    q = normalize(multiply(conjugate(normalize(current)), normalize(target)))
    # Canonical sign also makes the exact 180-degree ambiguity deterministic.
    if q[3] < 0 or (q[3] == 0 and next((v for v in q[:3] if v != 0), 0) < 0):
        q = tuple(-v for v in q)
    length = sqrt(sum(v*v for v in q[:3]))
    if length < SMALL_ROTATION_VECTOR_NORM:
        return tuple(2*v for v in q[:3])
    factor = 2 * atan2(length, q[3]) / length
    return tuple(v * factor for v in q[:3])


def yaw_from_quaternion(orientation: Quaternion) -> float:
    """World ZYX heading in radians from a body-to-world quaternion."""
    x, y, z, w = normalize(orientation)
    return atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def body_to_world(vector: Vector3, orientation: Quaternion) -> Vector3:
    """Rotate body XYZ into world XYZ without changing physical units."""
    vector = _values(vector, VECTOR_COMPONENTS, "vector")
    q = normalize(orientation)
    return multiply(multiply(q, (*vector, 0.0)), conjugate(q))[:3]
