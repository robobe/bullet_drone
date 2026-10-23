"""Simulation and recording lifecycle events; excludes mission phases."""

from ._common import BOOLEAN, topic_schema


TOPIC = "simulation/events"
SCHEMA = topic_schema(TOPIC, {
    "event": {"enum": ["reset", "recording_complete", "failure", "recording_gap"]},
    "details": {"type": "string"},
    "dropped_frames": {"type": "integer", "minimum": 0, "description": "Cumulative dropped-frame count for this recording"},
    "complete": {**BOOLEAN, "description": "False after data loss/failure; final completeness is reported at recording completion"},
})
