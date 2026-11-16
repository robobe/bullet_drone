from types import SimpleNamespace
from unittest.mock import Mock
from examples.wind_control.service import SimulationService
from bullet_simulation_protocol.requests import OperationReply


def test_dispatch_lifecycle_expiry_and_target_validation():
    service = SimulationService({'request_endpoint':'tcp://127.0.0.1:5560'}, clock=lambda:10.)
    service.io = Mock(error=None)
    observation = SimpleNamespace(ground_truth=SimpleNamespace(drone_id='drone_1'))
    fresh = SimpleNamespace(ground_truth=SimpleNamespace(drone_id='drone_1'))
    start, reset, spawn = Mock(), Mock(return_value=fresh), Mock(return_value=fresh)
    simulation = Mock()

    def call(topic, payload, age=0., drone_id='drone_1'):
        service.io.poll_request.return_value = dict(token='internal',received_at=10.-age,
            payload=dict(topic=topic,drone_id=drone_id,payload=payload))
        service.update(simulation,observation,replace_spawn=spawn,reset_simulation=reset,start_mission=start)
        return service.io.reply.call_args.args[1].to_dict()

    assert call('spawn',{'position_m':[0,0,3]}) == {'success':True}
    assert call('command',{'action':'start'}) == {'success':True}
    assert call('spawn',{'position_m':[0,0,3]})['reason'] == 'spawn_requires_stopped'
    assert call('command',{'action':'reset'})['reason'] == 'reset_requires_stopped'
    wind = dict(enabled=True,behavior='constant',velocity_world_m_s=[1,0,0],gust_stddev_m_s=[0,0,0],correlation_time_s=1.,seed=1)
    assert call('wind',wind) == {'success':True}
    assert call('command',{'action':'stop'}) == {'success':True}
    assert call('command',{'action':'start'})['reason'] == 'reset_required'
    assert call('command',{'action':'reset'}) == {'success':True}
    assert call('command',{'action':'reset'}) == {'success':True}
    assert reset.call_count == 2  # Repeated calls are independent.
    assert call('wind',wind) == {'success':True}
    assert call('command',{'action':'start'},age=2.)['reason'] == 'request_expired_in_queue'
    assert call('command',{'action':'start'},drone_id='other')['reason'] == 'unknown_drone'
