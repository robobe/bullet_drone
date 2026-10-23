"""Shared, transport-independent sample identity and simulation timestamp."""

from bullet_drone.models import MAX_DRONE_ID_LENGTH


SCHEMA_VERSION = 1
HEADER_SCHEMA = {
    "type": "object",
    "properties": {
        "schema_version": {"type": "integer", "const": SCHEMA_VERSION},
        "drone_id": {
            "type": "string", "minLength": 1, "maxLength": MAX_DRONE_ID_LENGTH,
            "pattern": r"\S", "description": "Stable configured vehicle identity",
        },
        "episode_id": {"type": "string", "minLength": 1, "description": "Reset/run identity"},
        "step_index": {"type": "integer", "minimum": 0},
        "simulation_time_s": {"type": "number", "minimum": 0, "description": "Simulation time in seconds"},
    },
    "required": ["schema_version", "drone_id", "episode_id", "step_index", "simulation_time_s"],
    "additionalProperties": False,
}
