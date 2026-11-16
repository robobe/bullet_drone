import json

import pytest
from jsonschema import validate
from bullet_simulation_protocol import schema_bytes
from bullet_simulation_protocol.control import validate_state
from bullet_simulation_protocol.scenario import validate_collision


def report():
    return dict(schema_version=1, event_id='hit', controller_id='godot', issued_us=1_010_000,
                source=dict(physics_id='physics', episode_id='episode', stream_id='stream',
                            step_index=2, simulation_time_s=.1, measurement_us=1_000_000),
                target_id='red_truck', drone_position_m=[50., 0., 5.])


def test_collision_and_lifecycle_contracts():
    assert validate_collision(report()) == report()
    validate(report(), json.loads(schema_bytes('collision.json')))
    state = dict(schema_version=1, physics_id='physics', controller_id='godot', episode_id='episode',
                 state='stopped', reason='completed', simulation_time_s=.12)
    assert validate_state(state) == state
    validate(state, json.loads(schema_bytes('state.json')))
    with pytest.raises(ValueError):
        validate_state(dict(state, schema_version=2, terminal_result={}))
    with pytest.raises(ValueError, match='unknown'):
        schema_bytes('scenario_end.json')


@pytest.mark.parametrize('changes', [dict(target_id='terrain'), dict(event_id='bad/id'), dict(issued_us=True),
    dict(issued_us=0), dict(drone_position_m=[0, 0]), dict(drone_position_m=[float('nan'), 0, 0])])
def test_malformed_report(changes):
    with pytest.raises(ValueError):
        validate_collision(dict(report(), **changes))
