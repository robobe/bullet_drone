import pytest
from bullet_simulation_protocol.requests import (
    SpawnPayload, SpawnRequest, WindRequest, CommandRequest, CommandPayload,
    OperationReply, parse_request,
)
from bullet_simulation_protocol.wind import WindSettingsMessage


def test_three_topic_roundtrips():
    calls = [SpawnRequest('drone_1', SpawnPayload((0., 0., 3.))),
             CommandRequest('drone_1', CommandPayload('start')),
             WindRequest('drone_1', WindSettingsMessage(True, 'constant', (1., 0., 0.), (0., 0., 0.), 1., 1))]
    for call in calls:
        assert parse_request(call.to_dict()) == call
        assert set(call.to_dict()) == {'topic', 'drone_id', 'payload'}
    assert OperationReply(True).to_dict() == {'success': True}
    failed = OperationReply(False, 'spawn_requires_stopped')
    assert OperationReply.from_dict(failed.to_dict()) == failed


@pytest.mark.parametrize('value', [
    {'topic':'get_wind','drone_id':'drone_1','payload':{}},
    {'topic':'command','drone_id':'drone_1','payload':{'action':'get_simulation_state'}},
    {'topic':'spawn','drone_id':'drone_1','payload':{'position_m':[0,0,3]},'request_id':'removed'},
    {'topic':'spawn','drone_id':'drone_1','payload':{'position_m':[0,0,float('nan')]}},
])
def test_obsolete_or_invalid_requests_rejected(value):
    with pytest.raises(ValueError):
        parse_request(value)


@pytest.mark.parametrize('reply', [{'success':True,'reason':'extra'}, {'success':False}, {'success':1}])
def test_reply_shape_is_strict(reply):
    with pytest.raises(ValueError):
        OperationReply.from_dict(reply)
