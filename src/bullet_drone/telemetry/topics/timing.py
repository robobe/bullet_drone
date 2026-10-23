"""Elapsed clocks and simulation-seconds per wall-clock second."""

from ._common import NONNEGATIVE, topic_schema


TOPIC = "simulation/timing"
SCHEMA = topic_schema(TOPIC, {
    "simulation_elapsed_s": NONNEGATIVE,
    "wall_elapsed_s": {**NONNEGATIVE, "description": "Elapsed monotonic wall time, not a UTC timestamp"},
    "real_time_factor": {"type": ["number", "null"], "minimum": 0,
                         "description": "simulation_elapsed_s / wall_elapsed_s; null before a positive wall interval exists"},
})
