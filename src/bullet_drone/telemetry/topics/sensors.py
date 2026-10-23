"""Typed sensor readings; disabled measurements are null."""

from ._common import NUMBER, SOURCE_PROPERTIES, VECTOR3, object_schema, topic_schema


TOPIC = "simulation/sensors"
IMU_SCHEMA = {
    **object_schema({
        "episode_id": {"type": "string", "minLength": 1},
        **SOURCE_PROPERTIES,
        "specific_force_m_s2": {**VECTOR3, "description": "Body XYZ specific force in m/s^2"},
        "angular_velocity_rad_s": {**VECTOR3, "description": "Body XYZ angular velocity in rad/s"},
    }),
    "type": ["object", "null"],
}
BAROMETER_SCHEMA = {
    **object_schema({
        "episode_id": {"type": "string", "minLength": 1},
        **SOURCE_PROPERTIES,
        "pressure_pa": {**NUMBER, "exclusiveMinimum": 0, "description": "Measured pressure in Pa"},
        "altitude_m": {**NUMBER, "description": "Pressure-derived world Z in metres"},
    }),
    "type": ["object", "null"],
}
SCHEMA = topic_schema(TOPIC, {
    "imu": IMU_SCHEMA,
    "barometer": BAROMETER_SCHEMA,
    "rangefinder": {"type": "null", "description": "Not implemented"},
})
