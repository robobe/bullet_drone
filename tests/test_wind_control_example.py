from dataclasses import asdict
from types import SimpleNamespace
from unittest.mock import Mock
import socket,time
import pytest
from bullet_drone.config import WindSettings
from bullet_drone.forces.wind import WindModel
from bullet_simulation_protocol.transport import ZmqIo
from bullet_simulation_protocol.wind import SetWindRequest, WindSettingsMessage, WindReply
from examples.wind_control.service import SimulationService

WAIT_S=3


def test_wind_real_requests_reject_stale_and_preserve_readback():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0))
        endpoint=f'tcp://127.0.0.1:{sock.getsockname()[1]}'
    model=WindModel(WindSettings(behavior='gusts',gust_stddev_m_s=(1.,1.,0.)))
    model.velocity(.5)
    simulation=Mock()
    type(simulation).wind=property(lambda self:model.snapshot)
    simulation.set_wind.side_effect=lambda settings:model.set_settings(settings,.5)
    obs=SimpleNamespace(ground_truth=SimpleNamespace(episode_id='episode',step_index=120))
    with SimulationService({'wind_endpoint':endpoint},'physics') as service,ZmqIo(request_endpoint=endpoint) as client:
        def transact(payload):
            future=client.request(payload)
            deadline=time.monotonic()+WAIT_S
            while not future.done() and time.monotonic()<deadline:
                service.update(simulation,obs)
                time.sleep(.002)
            return WindReply.from_dict(future.result(1)['payload'])
        before=model.snapshot
        assert transact({'action':'get_wind'}).status=='current'
        assert model.snapshot==before
        simulation.set_wind.assert_not_called()
        settings=WindSettingsMessage.from_dict(asdict(WindSettings(velocity_world_m_s=(2.,0.,0.))))
        request=SetWindRequest('physics','episode',int(time.monotonic()*1000),settings)
        reply=transact(request.to_dict())
        assert reply.status=='applied' and model.snapshot.velocity_world_m_s==(2.,0.,0.)
        for bad in [dict(request.to_dict(),episode_id='old'),dict(request.to_dict(),issued_ms=0),{'action':'wrong'}]:
            assert transact(bad).status=='rejected'
        assert simulation.set_wind.call_count==1


def test_simulation_command_lifecycle_and_reset_restriction():
    from bullet_simulation_protocol.control import CommandReply
    from unittest.mock import Mock
    service = SimulationService({'wind_endpoint': 'tcp://127.0.0.1:5560'}, 'physics', clock=lambda: 1.)
    service.io = Mock()
    current = SimpleNamespace(ground_truth=SimpleNamespace(episode_id='episode', simulation_time_s=0.))
    fresh = SimpleNamespace(ground_truth=SimpleNamespace(episode_id='fresh', simulation_time_s=0.))
    start = Mock()
    reset = Mock(return_value=fresh)

    def command(action, observation=current, **changes):
        payload = dict(action=action, physics_id='physics',
                       episode_id=observation.ground_truth.episode_id, issued_ms=1000)
        payload.update(changes)
        service._command(dict(request_id='request', payload=payload), observation, reset, start)
        reply = service.io.reply.call_args.args[1]
        assert isinstance(reply, CommandReply)
        return reply

    assert command('start').status == 'applied'
    assert command('start').status == 'current'
    assert command('reset').reason == 'reset_requires_stopped'
    reset.assert_not_called()
    assert command('stop').status == 'applied'
    assert command('stop').status == 'current'
    assert command('start').reason == 'reset_required'
    assert command('reset').state.episode_id == 'fresh'
    assert command('start', fresh).status == 'applied'
    service.complete()
    assert command('start', fresh).reason == 'reset_required'
    assert command('stop', fresh, physics_id='other').status == 'rejected'
    assert command('stop', fresh, issued_ms=1001).status == 'rejected'
