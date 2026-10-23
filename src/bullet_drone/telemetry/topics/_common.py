"""Schema fragments and builders; no collection, validation or output at runtime."""

from copy import deepcopy

from bullet_drone.telemetry.header import HEADER_SCHEMA


JSON_SCHEMA_DIALECT = "https://json-schema.org/draft/2020-12/schema"
VECTOR_SIZE = 3
ROTOR_COUNT = 4
RC_CHANNEL_COUNT = 18
RC_MIN_US = 1000
RC_MAX_US = 2000
NUMBER = {"type": "number"}
NONNEGATIVE = {"type": "number", "minimum": 0}
BOOLEAN = {"type": "boolean"}
VECTOR3 = {"type": "array", "items": NUMBER, "minItems": VECTOR_SIZE, "maxItems": VECTOR_SIZE}
ROTOR4 = {"type": "array", "items": NONNEGATIVE, "minItems": ROTOR_COUNT, "maxItems": ROTOR_COUNT}
QUATERNION = {"type": "array", "items": NUMBER, "minItems": ROTOR_COUNT, "maxItems": ROTOR_COUNT}
ROTOR_VECTORS = {"type": "array", "items": VECTOR3, "minItems": ROTOR_COUNT, "maxItems": ROTOR_COUNT}
SOURCE_PROPERTIES = {
    "step_index": {"type": "integer", "minimum": 0},
    "simulation_time_s": {**NONNEGATIVE, "description": "Source state simulation time in seconds"},
}


def object_schema(properties: dict, description: str = "") -> dict:
    """All declared fields are required; unavailable values must be explicitly null."""
    return {"type": "object", "description": description, "properties": properties,
            "required": list(properties), "additionalProperties": False}


def topic_schema(topic: str, properties: dict, *, sourced: bool = False) -> dict:
    """Build a self-contained schema with a private copy of shared fragments."""
    data = dict(properties)
    if sourced:
        data["source"] = object_schema(SOURCE_PROPERTIES, "Pre-step state used in this calculation; same episode/drone as header")
    return deepcopy({
        "$schema": JSON_SCHEMA_DIALECT,
        "title": topic,
        **object_schema({"header": HEADER_SCHEMA, "data": object_schema(data)}),
    })
