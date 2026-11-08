from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from bullet_drone.config import WindSettings
from examples.wind_control.panel import wind_settings, publish_wind, WindPanel
from bullet_simulation_protocol.wind import validate_state


@pytest.mark.parametrize('values', [('nan','0','0'), ('inf','0','0'), ('no','0','0')])
def test_nonfinite_and_invalid_wind_rejected(values):
    with pytest.raises(ValueError):
        wind_settings(True, values)


def test_wind_settings_and_state_match_protocol():
    settings = wind_settings(True, ('1', '-2', '0.5'))
    snapshot = SimpleNamespace(settings=settings, velocity_world_m_s=settings.velocity_world_m_s, simulation_time_s=2.0)
    simulation = SimpleNamespace(wind=snapshot)
    renderer = SimpleNamespace(physics_id='physics', publisher=Mock())
    observation = SimpleNamespace(ground_truth=SimpleNamespace(episode_id='episode'))
    for request in (None, 'request'):
        publish_wind(simulation, renderer, observation, request)
        packet = renderer.publisher.set_wind.call_args.args[0]
        validate_state(packet)
        assert packet['velocity_world_m_s'] == [1.0, -2.0, 0.5]
        assert packet['result'] == ('idle' if request is None else 'applied')


def test_panel_applies_at_step_boundary_and_close_interrupts():
    panel = WindPanel.__new__(WindPanel)
    panel.root = Mock()
    panel.closed = False
    panel.pending = WindSettings(velocity_world_m_s=(1,0,0))
    panel.request_id = None
    panel.next_update = 0
    panel.status = Mock()
    simulation = Mock()
    simulation.wind = SimpleNamespace(settings=panel.pending, velocity_world_m_s=(1,0,0), simulation_time_s=0)
    renderer = SimpleNamespace(physics_id='physics', publisher=Mock())
    observation = SimpleNamespace(ground_truth=SimpleNamespace(episode_id='episode'))
    panel.update(simulation, renderer, observation)
    simulation.set_wind.assert_called_once()
    assert panel.pending is None
    validate_state(renderer.publisher.set_wind.call_args.args[0])
    panel.closed = True
    panel.next_update = 0
    with pytest.raises(KeyboardInterrupt):
        panel.update(simulation, renderer, observation)
