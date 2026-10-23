"""Ground-truth pose, motion and actual actuator speeds."""

from ._common import QUATERNION, ROTOR4, VECTOR3, topic_schema


TOPIC = "simulation/ground_truth"
SCHEMA = topic_schema(TOPIC, {
    "position_m": {**VECTOR3, "description": "World XYZ position in metres, Z up"},
    "orientation_xyzw": {**QUATERNION, "description": "Body-to-world quaternion, XYZW order"},
    "linear_velocity_m_s": {**VECTOR3, "description": "World XYZ velocity in m/s"},
    "angular_velocity_rad_s": {**VECTOR3, "description": "World XYZ angular velocity in rad/s"},
    "rotor_speeds_rad_s": {**ROTOR4, "description": "Actual prop0..prop3 speeds in rad/s"},
})
