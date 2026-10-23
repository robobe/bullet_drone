"""Raw RC pulse-width channels, before decoding."""

from ._common import RC_CHANNEL_COUNT, RC_MAX_US, RC_MIN_US, topic_schema


TOPIC = "simulation/rc_input"
SCHEMA = topic_schema(TOPIC, {
    "channels": {"type": "array", "items": {"type": "integer", "minimum": RC_MIN_US, "maximum": RC_MAX_US},
                 "minItems": RC_CHANNEL_COUNT, "maxItems": RC_CHANNEL_COUNT,
                 "description": "CH1..CH18 RC pulse widths in microseconds; not normalized joystick axes"},
})
