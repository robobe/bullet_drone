"""External runner loads YAML and shares the sensor pose with app feedback."""
from dataclasses import asdict
from types import SimpleNamespace
from unittest.mock import Mock
import json

import pytest
import yaml

from bullet_drone.config import PoseSources, SensorsSettings, ImuSettings, BarometerSettings
from bullet_drone.pose import resolve_pose
from tools.zmq_render.publisher import RenderPublisher


@pytest.mark.parametrize('filter_enabled', [False, True])
def test_external_yaml_sensor_pose_reaches_renderer_and_app(config, tmp_path, monkeypatch, capsys, filter_enabled):
    from examples.external_rc import main as runner
    data = asdict(config)
    data['simulation']['gui'] = True  # The external runner remains headless.
    data['pose_sources'] = asdict(PoseSources('barometer', 'imu'))
    data['sensors'] = asdict(SensorsSettings(ImuSettings(gyro_bias_rad_s=(.1, .1, .1)),
                                           BarometerSettings(pressure_bias_pa=5., filter_enabled=filter_enabled)))
    physics = tmp_path/'physics.yaml'
    physics.write_text(yaml.safe_dump(data))
    transport = tmp_path/'transport.json'
    transport.write_text(json.dumps(dict(telemetry_endpoint='tcp://127.0.0.1:5555',
        image_endpoint='tcp://127.0.0.1:5556', control_endpoint='tcp://127.0.0.1:5557',
        flight_command_endpoint='tcp://127.0.0.1:5559')))
    render = tmp_path/'render.json'
    render.write_text(json.dumps(dict(transport_config='transport.json', fps=30,
                                     camera=dict(width_px=640, height_px=480, fov_x_deg=90))))
    output = RenderPublisher(render, pose_sources=PoseSources('barometer', 'imu'))
    output.thread = SimpleNamespace()
    output.start = Mock()
    output.close = Mock()
    output.set_flight = Mock(wraps=output.set_flight)
    output.consume = Mock(wraps=output.consume)
    factory = Mock(return_value=output)
    monkeypatch.setattr(runner, 'RenderPublisher', factory)
    monkeypatch.setattr(runner, 'FlightReceiver', Mock())
    lease = Mock(owner='app', candidate={}, sequence=1)
    lease.ready.return_value = True
    lease.command.return_value = [1500, 1500, 1000, 1500, 1000, 2000] + [1000] * 12
    monkeypatch.setattr(runner, 'FlightLease', Mock(return_value=lease))
    actions = iter(['start', None, 'stop', None])
    class Control:
        physics_id = 'physics'
        controller_id = None
        state, reason = 'stopped', 'waiting_app'
        def __init__(self, path, renderer, *, scenario, spawn, wind):
            self.spawn = spawn
        def transition(self, state, reason, observation):
            self.state, self.reason = state, reason
        def poll(self):
            try:
                return next(actions)
            except StopIteration:
                raise KeyboardInterrupt
        def publish(self, observation, **kwargs):
            pass
        def close(self):
            pass
    monkeypatch.setattr(runner, 'SimulationControl', Control)
    assert runner.CliParser().parse(['--config', str(physics), '--render-config', str(render)]) == (render, physics)
    with pytest.raises(KeyboardInterrupt):
        runner.run(render, config_path=physics)
    stdout = capsys.readouterr().out
    assert stdout.count('=== bullet_drone loaded configuration ===') == 1
    banner = json.loads(stdout.split('\n', 1)[1])
    assert banner['physics_config_path'] == str(physics.resolve())
    expected = yaml.safe_load(physics.read_text())
    expected['simulation']['gui'] = False
    assert banner['simulation'] == expected
    assert banner['flight_control'] == json.loads(json.dumps(asdict(runner.FLIGHT_CONTROL_SETTINGS)))
    assert banner['vehicle']['rotor_positions_m']
    assert banner['render_config_path'] == str(render.resolve())
    assert banner['render'] == output.config
    assert banner['transport_config_path'] == str(transport.resolve())
    assert banner['transport'] == json.loads(transport.read_text())
    assert banner['platform']['attitude_model'] == 'imu_complementary'
    factory.assert_called_once_with(render, pose_sources=PoseSources('barometer', 'imu'))
    observations = output.consume.call_args_list
    feedbacks = output.set_flight.call_args_list
    assert len(observations) == 4  # Initial, new episode, two physics steps.
    for capture, feedback in zip(observations, feedbacks):
        observation = capture.args[0]
        platform, packet = feedback.args
        assert banner['platform'] == platform
        expected = resolve_pose(observation, output.pose_sources)
        assert platform['attitude_model'] == 'imu_complementary'
        assert packet['orientation_xyzw'] == list(expected.orientation_xyzw)
        assert packet['barometer_altitude_m'] == expected.position_m[2]
        assert packet['source'] == capture.kwargs['source']
    assert observations[0].args[0].ground_truth.episode_id != observations[1].args[0].ground_truth.episode_id
    assert feedbacks[-1].args[1]['orientation_xyzw'] == feedbacks[-2].args[1]['orientation_xyzw']
    assert feedbacks[-1].args[1]['source'] == feedbacks[-2].args[1]['source']
    assert feedbacks[-1].args[1]['state'] == 'stopped'
    assert output.latest_pose.orientation_xyzw != (0., 0., 0., 1.)
    output.close.assert_called_once()
