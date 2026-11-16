import json

import pytest

from bullet_simulation_protocol.spawn import SpawnRequest, SpawnState, validate_request, validate_state


def request():
    return dict(schema_version=2, physics_id='physics', episode_id='episode', controller_id='godot',
                action='set_spawn', takeoff_height_m=7.5, request_id='apply1', issued_ms=1000, scenario_name='Hill', position_m=[12, -34, 25.125])


def state():
    return dict(schema_version=2, physics_id='physics', episode_id='episode', controller_id=None,
                takeoff_height_m=7.5, request_id=None, result='idle', reason='startup', scenario_name='Default', position_m=[0, 0, .015])


def test_shared_spawn_dataclasses():
    for packet, cls in ((request(), SpawnRequest), (state(), SpawnState)):
        assert cls.from_dict(packet).to_dict() == packet


@pytest.mark.parametrize('change', [dict(schema_version=True), dict(issued_ms=True), dict(request_id='a/b'),
    dict(scenario_name=' '), dict(scenario_name=' bad'), dict(scenario_name='a\nb'), dict(position_m=[0, 0, float('nan')]),
    dict(position_m=[251, 0, 1]), dict(position_m=[0, -251, 1]), dict(position_m=[0, 0, 0]),
    dict(position_m=[0, 0, 501]), dict(position_m=[0, True, 10]), dict(extra=1)])
def test_bad_spawn_requests(change):
    with pytest.raises(ValueError):
        validate_request(dict(request(), **change))


@pytest.mark.parametrize('change', [dict(result='applied'), dict(result='other'), dict(request_id='request'),
    dict(controller_id='bad/id'), dict(reason=''), dict(position_m=[0, 0, float('inf')])])
def test_bad_spawn_state(change):
    with pytest.raises(ValueError):
        validate_state(dict(state(), **change))


@pytest.mark.parametrize('factory,validator,name', [(request, validate_request, 'spawn_request.json'),
                                                   (state, validate_state, 'spawn_state.json')])
def test_spawn_v2_takeoff(factory, validator, name):
    packet = dict(factory(), schema_version=2, takeoff_height_m=7.5)
    validator(packet)
    for value in (0, -1, 100.1, True, float('nan'), float('inf'), '6'):
        with pytest.raises(ValueError):
            validator(dict(packet, takeoff_height_m=value))
    packet.pop('takeoff_height_m')
    with pytest.raises(ValueError):
        validator(packet)
