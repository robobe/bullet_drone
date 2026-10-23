"""Simulation topic registry and pure configuration-based selection."""

from bullet_drone.config import TelemetrySettings
from . import airflow, allocation, config, control, events, forces, ground_truth, rc_input, sensors, timing


TOPIC_SCHEMAS = {module.TOPIC: module.SCHEMA for module in (
    ground_truth, config, events, sensors, rc_input, control, allocation, forces, airflow, timing,
)}
MANDATORY_TOPICS = frozenset((ground_truth.TOPIC, config.TOPIC, events.TOPIC))
OPTIONAL_TOPICS = {
    "sensors": sensors.TOPIC, "rc_input": rc_input.TOPIC, "control": control.TOPIC,
    "allocation": allocation.TOPIC, "forces": forces.TOPIC, "airflow": airflow.TOPIC,
    "timing": timing.TOPIC,
}


def selected_schemas(settings: TelemetrySettings) -> dict[str, dict]:
    """Return mandatory and enabled optional schemas; starts no recording resources."""
    selected = MANDATORY_TOPICS | {topic for flag, topic in OPTIONAL_TOPICS.items() if getattr(settings, flag)}
    return {topic: schema for topic, schema in TOPIC_SCHEMAS.items() if topic in selected}
