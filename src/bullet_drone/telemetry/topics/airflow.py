"""Ambient wind, local wakes and air-relative velocities in explicit frames."""

from ._common import ROTOR_VECTORS, VECTOR3, topic_schema


TOPIC = "simulation/airflow"
SCHEMA = topic_schema(TOPIC, {
    "ambient_wind_world_m_s": {**VECTOR3, "description": "Total world XYZ ambient air velocity, mean plus gust, m/s"},
    "gust_world_m_s": {**VECTOR3, "description": "World XYZ gust component, m/s; zero for constant wind"},
    "body_wake_velocity_body_m_s": {**VECTOR3, "description": "Local wake air velocity at body sample point in body axes, m/s"},
    "rotor_wake_velocities_body_m_s": {**ROTOR_VECTORS, "description": "Wake air velocities at prop0..prop3 in body axes, m/s"},
    "body_relative_velocity_body_m_s": {**VECTOR3, "description": "Body velocity minus local air velocity in body axes, m/s"},
    "rotor_relative_velocities_body_m_s": {**ROTOR_VECTORS, "description": "Prop0..prop3 velocities minus local air velocity in body axes, m/s"},
}, sourced=True)
