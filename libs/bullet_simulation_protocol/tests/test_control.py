import json
import time

import pytest
from jsonschema import validate
from bullet_simulation_protocol import schema_bytes
from bullet_simulation_protocol.control import validate_command, decode_json


def command(**changes):
    return dict(physics_id='physics', episode_id='episode',
                action='start', issued_ms=int(time.monotonic()*1000), **changes)


def test_command_and_state_contracts():
    value = command()
    assert validate_command(value) == value
    from bullet_simulation_protocol.control import SimulationCommand
    assert SimulationCommand.from_dict(value).to_dict() == value
    with pytest.raises(ValueError):
        schema_bytes('command.json')
    with pytest.raises(ValueError):
        schema_bytes('state.json')


@pytest.mark.parametrize('changes', [dict(schema_version=True), dict(sequence=0), dict(issued_ms=True),
    dict(action='pause'), dict(issued_ms=-1), dict(physics_id='bad/id'), dict(extra=1)])
def test_bad_command(changes):
    value = command()
    value.update(changes)
    with pytest.raises(ValueError):
        validate_command(value)


@pytest.mark.parametrize('payload', [b'', b'{}'*3000, b'\xff', b'{"value":NaN}'])
def test_bad_json(payload):
    with pytest.raises((ValueError, UnicodeError)):
        decode_json(payload)


def test_command_reply_roundtrip():
    from bullet_simulation_protocol.control import CommandReply, SimulationStatus, GetSimulationState
    state = SimulationStatus('physics', 'episode', 'stopped', 'reset', 0., True)
    reply = CommandReply('applied', 'reset', state)
    assert CommandReply.from_dict(reply.to_dict()) == reply
    assert GetSimulationState.from_dict({'action': 'get_simulation_state'}) == GetSimulationState()
