"""Mixer prediction before motor dynamics and aerodynamic corrections."""

from ._common import BOOLEAN, NONNEGATIVE, ROTOR4, VECTOR3, object_schema, topic_schema


TOPIC = "simulation/allocation"
WRENCH_SCHEMA = object_schema({
    "thrust_n": {**NONNEGATIVE, "description": "Collective body +Z thrust in N"},
    "torque_nm": {**VECTOR3, "description": "Body XYZ torque in N*m"},
})
ALLOCATION_SCHEMA = object_schema({
    "requested": WRENCH_SCHEMA,
    "achieved": {**WRENCH_SCHEMA, "description": "Ideal mixer prediction, not delivered aerodynamic wrench"},
    "speeds_rad_s": {**ROTOR4, "description": "Target prop0..prop3 speeds before motor dynamics, rad/s"},
    "rotor_thrusts_n": {**ROTOR4, "description": "Ideal prop0..prop3 thrusts in N"},
    "saturated": BOOLEAN,
    "torque_scale": {"type": "number", "minimum": 0, "maximum": 1},
})
SCHEMA = topic_schema(TOPIC, {
    "allocation": {**ALLOCATION_SCHEMA, "type": ["object", "null"], "description": "Null for disarmed steps with no allocation"},
})
