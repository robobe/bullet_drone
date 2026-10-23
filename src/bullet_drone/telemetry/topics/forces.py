"""Modeled forces only; PyBullet owns gravity, contacts and rotor lever-arm torque."""

from ._common import ROTOR4, ROTOR_VECTORS, VECTOR3, object_schema, topic_schema


TOPIC = "simulation/forces"
FORCE_OUTPUT_SCHEMA = object_schema({
    "rotor_forces_n": {**ROTOR_VECTORS, "description": "Body XYZ forces applied at prop0..prop3, N"},
    "body_force_n": {**VECTOR3, "description": "Body XYZ force applied at COM, N"},
    "body_torque_nm": {**VECTOR3, "description": "Explicit body XYZ torque, N*m; excludes rotor-position cross force"},
})
SCHEMA = topic_schema(TOPIC, {
    "total": FORCE_OUTPUT_SCHEMA,
    "contributions": {"type": "object", "additionalProperties": FORCE_OUTPUT_SCHEMA,
                      "description": "Named additive model contributions; sum equals total. Apply multipliers to baseline thrust before decomposing"},
    "inflow_multipliers": {**ROTOR4, "description": "Dimensionless prop0..prop3 thrust multipliers"},
    "ground_effect_multipliers": {**ROTOR4, "description": "Dimensionless prop0..prop3 thrust multipliers"},
}, sourced=True)
