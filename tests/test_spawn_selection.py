"""Spawn transaction tests over real lifecycle TCP, including retry after episode change."""
import json
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import zmq

from bullet_msgs.control import HEARTBEAT_TOPIC
from bullet_msgs.spawn import REQUEST_TOPIC
from tools.zmq_render.control import SimulationControl
from tools.zmq_render.spawn import SpawnSelection


@pytest.fixture
def wire(tmp_path):
    with zmq.Context() as context, context.socket(zmq.XPUB) as publisher:
        publisher.setsockopt(zmq.LINGER, 0)
        publisher.bind_to_random_port('tcp://127.0.0.1')
        path = tmp_path / 'transport.json'
        path.write_text(json.dumps(dict(control_endpoint=publisher.getsockopt_string(zmq.LAST_ENDPOINT),
                                       telemetry_endpoint='tcp://127.0.0.1:5555', image_endpoint='tcp://127.0.0.1:5556')))
        clock = [time.monotonic()]
        control = SimulationControl(path, Mock(), clock=lambda: clock[0], spawn=SpawnSelection((0, 0, .015)))
        observation = SimpleNamespace(ground_truth=SimpleNamespace(episode_id='first', simulation_time_s=0))
        control.publish(observation)
        for _ in range(3):
            assert publisher.poll(2000)
            publisher.recv()
        def send(packet, topic=REQUEST_TOPIC):
            publisher.send_multipart([topic, json.dumps(packet).encode()])
            assert control.socket.poll(2000)
            return control.poll()
        heartbeat = dict(schema_version=1, controller_id='godot', sequence=1, action='heartbeat', issued_ms=int(clock[0]*1000))
        assert send(heartbeat, HEARTBEAT_TOPIC) is None
        packet = dict(schema_version=1, physics_id=control.physics_id, episode_id='first', controller_id='godot',
                      request_id='pick1', issued_ms=int(clock[0]*1000), scenario_name='Hill', position_m=[20, -30, 40])
        try:
            yield control, clock, send, packet, observation
        finally:
            control.close()


def test_spawn_retry_does_not_reset_new_episode_or_reapply_old_config(wire):
    control, _, send, packet, observation = wire
    assert send(packet, REQUEST_TOPIC+b'/suffix') is None
    assert send(packet) == 'spawn'
    control.spawn.finish(True, 'spawn_applied')
    observation.ground_truth.episode_id = 'second'
    control.publish(observation)
    assert send(packet) is None  # Lost ACK: the original episode is now old, still ack it.
    assert control.spawn.result == 'applied'
    assert control.spawn.pending is None
    assert control.spawn.position == [20, -30, 40]
    assert send(dict(packet, position_m=[50, 60, 70])) is None
    assert control.spawn.position == [20, -30, 40]
    assert send(dict(packet, request_id='pick2', episode_id='first')) is None
    assert control.spawn.reason == 'stale_episode'
    assert send(dict(packet, request_id='pick3', episode_id='second')) == 'spawn'
    control.spawn.finish(False, 'initialization_failed')
    assert control.spawn.position == [20, -30, 40]


@pytest.mark.parametrize('changes', [dict(physics_id='foreign'), dict(controller_id='foreign'), dict(issued_ms=0),
                                    dict(position_m=[0, 0, -1]), dict(schema_version=2)])
def test_spawn_rejects_foreign_stale_malformed(wire, changes):
    control, _, send, packet, _ = wire
    assert send(dict(packet, **changes)) is None
    assert control.spawn.pending is None
    assert control.spawn.position == [0, 0, .015]


def test_running_and_heartbeat_timeout_rejected(wire):
    control, clock, send, packet, _ = wire
    control.state = 'running'
    assert send(packet) is None
    assert control.spawn.reason == 'physics_running'
    control.state = 'stopped'
    clock[0] += 4
    assert send(dict(packet, request_id='new', issued_ms=int(clock[0]*1000))) is None
    assert control.spawn.position == [0, 0, .015]


@pytest.mark.parametrize("cleanup_fails", [False, True])
def test_runner_replaces_only_after_success_and_reset_keeps_spawn(tmp_path, monkeypatch, cleanup_fails, capsys):
    from examples.external_rc import main as runner
    from bullet_drone.simulation import RcSimulation
    transport = tmp_path / 'transport.json'
    transport.write_text(json.dumps(dict(telemetry_endpoint='tcp://127.0.0.1:5555', image_endpoint='tcp://127.0.0.1:5556',
                                        control_endpoint='tcp://127.0.0.1:5557', flight_command_endpoint='tcp://127.0.0.1:5559')))
    publisher = Mock(transport_path=transport, stream='stream', config={'camera':dict(width_px=2, height_px=2, fov_x_deg=90)})
    monkeypatch.setattr(runner, 'RenderPublisher', lambda path, **kwargs: publisher)
    monkeypatch.setattr(runner, 'FlightReceiver', Mock())
    states, instances, closed = [], [], []
    class Simulation:
        def __init__(self, config, settings, vehicle):
            self.inner = RcSimulation(config, settings, vehicle)
            self.position = config.drone.initial_position_m
            instances.append(self)
        @property
        def wind(self):
            return self.inner.wind
        def set_wind(self, settings):
            self.inner.set_wind(settings)
        def reset(self):
            if len(instances) >= 2 and self is instances[1]:  # First candidate fails during initialization.
                raise RuntimeError('injected candidate reset failure')
            return self.inner.reset()
        def close(self):
            closed.append(self)
            self.inner.close()
            if cleanup_fails and self in instances[:2]:
                raise RuntimeError("injected retired simulation cleanup failure")
    class Control:
        physics_id = 'physics'
        controller_id = None
        state = 'stopped'
        reason = 'startup'
        count = 0
        def __init__(self, path, renderer, *, scenario, spawn, wind):
            self.spawn = spawn
            self.wind = wind
        def transition(self, state, reason, observation):
            self.state, self.reason = state, reason
            states.append((reason, observation.ground_truth))
        def publish(self, observation, **kwargs):
            pass
        def poll(self):
            self.count += 1
            if self.count == 1:
                from dataclasses import asdict
                from bullet_drone.config import WindSettings
                self.wind.pending = dict(controller_id="godot", request_id="wind", settings=asdict(WindSettings(velocity_world_m_s=(3., -1., 0.))))
                return "wind"
            if self.count in (2, 3):
                self.spawn.pending = dict(schema_version=1, physics_id='physics', episode_id='episode', controller_id='godot',
                    request_id=f'pick{self.count}', issued_ms=1, scenario_name='Hill', position_m=[30, -20, 40])
                if self.count == 3:
                    assert instances[0] not in closed
                    assert self.spawn.reason == 'initialization_failed'
                    assert instances[0].reset().ground_truth.position_m == (0, 0, .015)
                return 'spawn'
            if self.count == 4:
                assert instances[0] in closed and instances[1] in closed
                return 'reset'
            raise KeyboardInterrupt
        def close(self):
            pass
    monkeypatch.setattr(runner, 'SimulationControl', Control)
    with pytest.raises(KeyboardInterrupt):
        runner.run('unused', simulation_factory=Simulation)
    stdout = capsys.readouterr().out
    assert stdout.count('=== bullet_drone loaded configuration ===') == 1
    banner = json.loads(stdout.split('\n', 1)[1])
    assert banner['physics_config_path'] is None
    assert banner['simulation']['pose_sources'] == dict(altitude='ground_truth', attitude='ground_truth')
    assert banner['simulation']['sensors']['imu']['gyro_stddev_rad_s'] == [0., 0., 0.]
    assert len(instances) == 3
    assert all(value in closed for value in instances)
    assert all(value.wind.settings.velocity_world_m_s == (3., -1., 0.) for value in instances)
    initial, applied, reset = [state for _, state in states]
    assert initial.position_m == (0, 0, .015)
    assert applied.position_m == reset.position_m == (30, -20, 40)
    assert applied.step_index == reset.step_index == 0
    assert applied.linear_velocity_m_s == (0, 0, 0)
    assert applied.orientation_xyzw == (0, 0, 0, 1)
    assert len({initial.episode_id, applied.episode_id, reset.episode_id}) == 3


def test_takeoff_selection_is_atomic_and_part_of_retry_identity(wire):
    control, _, send, packet, observation = wire
    packet = dict(packet, schema_version=2, takeoff_height_m=6.5)
    assert send(packet) == 'spawn'
    control.spawn.finish(True, 'spawn_applied')
    observation.ground_truth.episode_id = 'second'
    control.publish(observation)
    assert control.spawn.takeoff_height_m == 6.5
    assert send(dict(packet, takeoff_height_m=9)) is None
    assert control.spawn.takeoff_height_m == 6.5
    assert send(dict(packet, request_id='new', episode_id='second', takeoff_height_m=9)) == 'spawn'
    control.spawn.finish(False, 'initialization_failed')
    assert control.spawn.takeoff_height_m == 6.5
    # A legacy v1 Apply restores the default climb along with its position.
    legacy = {k: v for k, v in packet.items() if k != 'takeoff_height_m'}
    legacy.update(schema_version=1, request_id='legacy', episode_id='second')
    assert send(legacy) == 'spawn'
    control.spawn.finish(True, 'spawn_applied')
    assert control.spawn.takeoff_height_m == 20
