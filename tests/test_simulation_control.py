"""Real PUB/SUB fanout and deterministic control lifecycle at example loop boundaries."""
import importlib
import json
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import zmq

from bullet_simulation_protocol.control import COMMAND_TOPIC, HEARTBEAT_TOPIC
from tools.zmq_render.control import SimulationControl


@pytest.fixture
def wire(tmp_path):
    with zmq.Context() as context, context.socket(zmq.XPUB) as publisher:
        publisher.setsockopt(zmq.LINGER, 0)
        publisher.setsockopt(zmq.XPUB_VERBOSE, 1)
        publisher.bind_to_random_port('tcp://127.0.0.1')
        profile = dict(telemetry_endpoint='tcp://127.0.0.1:5555', image_endpoint='tcp://127.0.0.1:5556',
                       control_endpoint=publisher.getsockopt_string(zmq.LAST_ENDPOINT))
        path = tmp_path/'transport.json'
        path.write_text(json.dumps(profile))
        now = [time.monotonic()]
        controls = [SimulationControl(path, Mock(), clock=lambda: now[0]) for _ in range(2)]
        try:
            for _ in range(4):
                assert publisher.poll(5000)
                publisher.recv()
            def send(action, sequence, controller='godot', *, age=0, topic=None):
                value = dict(schema_version=1, controller_id=controller, sequence=sequence,
                             action=action, issued_ms=int((now[0]-age)*1000))
                publisher.send_multipart([topic or (HEARTBEAT_TOPIC if action=='heartbeat' else COMMAND_TOPIC), json.dumps(value).encode()])
                for control in controls:
                    assert control.socket.poll(2000)
            yield controls, now, send
        finally:
            for control in controls:
                control.close()


def test_fanout_duplicates_stale_topics_and_heartbeat_loss(wire):
    controls, now, send = wire
    send('heartbeat', 1)
    assert [c.poll() for c in controls] == [None, None]
    send('start', 2, topic=COMMAND_TOPIC+b'/extra')
    assert [c.poll() for c in controls] == [None, None]
    send('start', 2)
    assert [c.poll() for c in controls] == ['start', 'start']
    for c in controls:
        c.state = 'running'
    send('start', 2)
    assert [c.poll() for c in controls] == [None, None]
    send('reset', 3, age=2)
    assert [c.poll() for c in controls] == [None, None]
    send('stop', 4, topic=HEARTBEAT_TOPIC)
    assert [c.poll() for c in controls] == [None, None]
    now[0] += 3.1
    assert [c.poll() for c in controls] == ['controller_timeout', 'controller_timeout']
    for c in controls:
        c.state = 'stopped'
    send('heartbeat', 5)
    assert [c.poll() for c in controls] == [None, None]
    send('start', 6)
    assert [c.poll() for c in controls] == ['start', 'start']


def test_new_controller_stops_old_run_without_replaying_start(wire):
    controls, _, send = wire
    send('heartbeat', 1)
    for c in controls:
        c.poll()
        c.state = 'running'
    send('heartbeat', 1, controller='new-godot')
    assert [c.poll() for c in controls] == ['controller_changed', 'controller_changed']
    send('start', 2, controller='godot')
    assert [c.poll() for c in controls] == [None, None]


@pytest.mark.parametrize('example', ['position_square'])
def test_example_waits_stops_resets_and_starts_fresh(example, monkeypatch):
    demo = importlib.import_module(f'examples.{example}.demo')
    original = demo.RcSimulation
    samples = []
    transitions = []
    events = iter([None, None, 'start', None, 'start', 'stop', None, 'reset', None, 'start', None, 'controller_timeout'])
    class Control:
        state = 'stopped'
        def poll(self):
            try:
                return next(events)
            except StopIteration:
                raise KeyboardInterrupt
        def transition(self, state, reason, observation):
            self.state = state
            transitions.append((reason, observation.ground_truth))
        def publish(self, observation):
            pass
    instances = []
    def simulation(*args, **kwargs):
        value = original(*args, **kwargs)
        instances.append(value)
        return value
    monkeypatch.setattr(demo, 'RcSimulation', simulation)
    with pytest.raises(KeyboardInterrupt):
        demo.run_demo(gui=False, control=Control(), on_observation=samples.append)
    reasons = [reason for reason,_ in transitions]
    assert reasons == ['startup','started','stop','reset','started','controller_timeout']
    initial, started, stopped, reset, restarted, final = [state for _,state in transitions]
    assert initial.step_index == reset.step_index == restarted.step_index == 0
    assert stopped.step_index == 3  # Repeated Start while running does not reset.
    assert final.step_index == 2
    assert len({initial.episode_id, started.episode_id, reset.episode_id, restarted.episode_id}) == 4
    assert reset.position_m == initial.position_m
    assert instances[0]._closed


@pytest.mark.parametrize('example', ['position_square'])
@pytest.mark.parametrize('outcome', ['complete','error'])
def test_controlled_completion_and_failure_stay_available(example, outcome, monkeypatch):
    demo = importlib.import_module(f'examples.{example}.demo')
    mission_type = demo.SquareMission
    original = mission_type.command
    def command(self, observation, dt):
        if observation.ground_truth.step_index == 2:
            if outcome == 'error':
                raise RuntimeError('injected mission failure')
            self.phase = demo.Phase.COMPLETE
            return None
        return original(self, observation, dt)
    monkeypatch.setattr(mission_type, 'command', command)
    states = []
    class Control:
        state = 'stopped'
        def poll(self):
            if not states or states[-1][1] == 'startup':
                return 'start'
            if self.state != 'running':
                raise KeyboardInterrupt
        def transition(self, state, reason, observation):
            self.state = state
            states.append((state, reason))
        def publish(self, observation):
            pass
    with pytest.raises(KeyboardInterrupt):
        demo.run_demo(gui=False, control=Control())
    assert states[-1] == (('stopped','completed') if outcome=='complete' else ('error','runtime_failure'))


def test_fresh_queued_heartbeat_prevents_false_timeout_after_busy_physics(wire):
    controls, now, send = wire
    send('heartbeat', 1)
    for control in controls:
        control.poll()
        control.state = 'running'
    now[0] += 3.1
    send('heartbeat', 2)
    assert [control.poll() for control in controls] == [None, None]
    assert all(control.controller_id == 'godot' for control in controls)
