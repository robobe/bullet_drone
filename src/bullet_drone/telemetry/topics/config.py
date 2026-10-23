"""Initialized episode configuration snapshot; not a mission description."""

from ._common import topic_schema


TOPIC = "simulation/config"
SCHEMA = topic_schema(TOPIC, {
    "simulation_config": {"type": "object", "description": "JSON-compatible snapshot of the initialized SimulationConfig, including sensor/force/wind settings"},
    "vehicle_parameters": {"type": "object", "description": "Initialized VehicleParameters: body rotor positions (m), signs, thrust/torque coefficients and limits (N)"},
    "stabilization_settings": {"type": "object", "description": "Initialized InternalControlSettings: attitude/rate gains, limits and feedforward; SI units"},
    "seeds": {"type": "object", "additionalProperties": {"type": "integer"}, "description": "Named random seeds used by this episode"},
    "git_commit": {"type": ["string", "null"], "minLength": 1, "description": "Source revision when known; null for unavailable build metadata"},
})
