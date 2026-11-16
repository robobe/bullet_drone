from dataclasses import replace
import pytest
from bullet_simulation_protocol.wind import WindSettingsMessage, GetWindRequest, SetWindRequest, WindReadback, WindReply
from bullet_simulation_protocol import schema_bytes


def settings():
    return WindSettingsMessage(True,'gusts',(1.,2.,0.),(.1,.2,0.),2.,7)


def test_wind_dataclass_round_trips():
    values=[settings(),GetWindRequest(),SetWindRequest('physics','episode',1000,settings()),
            WindReadback('physics','episode',1,.1,settings(),(1.,2.,0.))]
    values.append(WindReply('current','readback',values[-1]))
    for value in values:
        assert type(value).from_dict(value.to_dict())==value
    for old in ('wind_request.json','wind_state.json'):
        with pytest.raises(ValueError):schema_bytes(old)


@pytest.mark.parametrize('key,value',[('enabled',1),('behavior','dryden'),('gust_stddev_m_s',(-1,0,0)),
    ('velocity_world_m_s',(0,float('nan'),0)),('correlation_time_s',0),('seed',True),('seed',.5)])
def test_bad_wind_settings(key,value):
    with pytest.raises(ValueError):replace(settings(),**{key:value})
