import pytest
from bullet_simulation_protocol import schema_bytes, validate_transport


def test_removed_rc_schema_and_endpoint_are_rejected():
    with pytest.raises(ValueError, match="unknown"):
        schema_bytes("flight_rc.json")
    with pytest.raises(ValueError):
        validate_transport(dict(telemetry_endpoint="tcp://127.0.0.1:5555",
                                image_endpoint="tcp://127.0.0.1:5556",
                                control_endpoint="tcp://127.0.0.1:5557",
                                flight_command_endpoint="tcp://127.0.0.1:5559"))
