"""Decoded angle or Acro requests used for a completed step."""

from ._common import BOOLEAN, NONNEGATIVE, VECTOR3, topic_schema


TOPIC = "simulation/control"
SCHEMA = topic_schema(TOPIC, {
    "mode": {"enum": ["angle", "acro"]},
    "armed": BOOLEAN,
    "rpy": {**VECTOR3, "description": "Angle: requested ZYX Euler angles in rad; Acro: requested body XYZ rates in rad/s"},
    "yaw_rate_rad_s": {"type": ["number", "null"], "description": "Angle-mode yaw-rate override in rad/s; null when unused"},
    "thrust_n": {**NONNEGATIVE, "description": "Requested collective thrust in N"},
}, sourced=True)
