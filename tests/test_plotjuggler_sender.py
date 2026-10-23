from dataclasses import replace
import json
import multiprocessing
from pathlib import Path
from queue import Empty, Full
import socket
import time
from unittest.mock import Mock

import pytest
from jsonschema import Draft202012Validator
import yaml

from bullet_drone.config import ConfigurationError
from bullet_drone.models import DroneObservation, DroneState
from bullet_drone.telemetry.topics.ground_truth import SCHEMA, TOPIC
from examples.position_square.demo_cli import CliParser
from tools.telemetry_sender.config import SenderSettings, load_settings
from tools.telemetry_sender.sender import TelemetrySender
from tools.telemetry_sender.worker import encode_datagram, send_loop


TEST_PORT = 12345
TEST_RATE_HZ = 5.0
TEST_INTERVAL_S = 1 / TEST_RATE_HZ
TEST_STEPS = 12
TEST_STEP_S = 1 / 240
TEST_WAIT_S = 2.0
TEST_CLOSE_MARGIN_S = 1.5
CONFIG_PATH = Path('tools/telemetry_sender/config.yaml')


def observation(step=0):
    return DroneObservation(DroneState('episode', step, step * TEST_STEP_S,
        (step * TEST_STEP_S, 0.0, 3.0), (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), drone_id='drone-test'))


def test_config_defaults_and_cli(tmp_path):
    assert load_settings(CONFIG_PATH) == SenderSettings()
    path = tmp_path / 'sender.yaml'
    path.write_text('{}')
    assert load_settings(path) == SenderSettings()
    options = CliParser().parse(['--telemetry-config', str(path), '--headless'])
    assert options.telemetry_config == path and options.headless
    path.write_text(yaml.safe_dump({'send_rate_hz': TEST_RATE_HZ, 'port': TEST_PORT}))
    assert load_settings(path).port == TEST_PORT


@pytest.mark.parametrize('key,value', [
    ('host', 'localhost'), ('host', 'not-an-address'), ('host', 1), ('host', None),
    ('host', '::1'), ('port', 0), ('port', 65536), ('port', True), ('port', 1.5),
    ('queue_capacity', 0), ('queue_capacity', True), ('queue_capacity', 1.5),
    ('send_rate_hz', 0), ('send_rate_hz', -1), ('send_rate_hz', True),
    ('send_rate_hz', float('inf')), ('send_rate_hz', float('nan')), ('send_rate_hz', '5'),
    ('startup_timeout_s', 0), ('shutdown_timeout_s', float('inf')),
])
def test_invalid_config_values(tmp_path, key, value):
    with pytest.raises(ConfigurationError):
        SenderSettings(**{key: value})
    path = tmp_path / 'sender.yaml'
    path.write_text(yaml.safe_dump({key: value}))
    with pytest.raises(ConfigurationError):
        load_settings(path)


@pytest.mark.parametrize('text', ['null', '[]', 'unknown: true', 'port: ['])
def test_invalid_yaml(tmp_path, text):
    path = tmp_path / 'sender.yaml'
    path.write_text(text)
    with pytest.raises(ConfigurationError):
        load_settings(path)
    with pytest.raises(ConfigurationError):
        load_settings(tmp_path / 'absent.yaml')


def test_wire_format_uses_existing_schema():
    state = observation(TEST_STEPS).ground_truth
    message = json.loads(encode_datagram(state))
    assert message['topic'] == TOPIC
    assert message['timestamp'] == state.simulation_time_s == message['sample']['header']['simulation_time_s']
    Draft202012Validator(SCHEMA).validate(message['sample'])
    assert message['sample']['data']['position_m'] == list(state.position_m)


class ScheduledQueue:
    """A virtual-clock queue lets us test rate control without scheduler timing."""

    def __init__(self, items):
        self.items = list(items)
        self.now = 0.0

    def get(self, timeout=None):
        if self.items and (timeout is None or self.items[0][0] <= self.now + timeout):
            when, item = self.items.pop(0)
            self.now = max(self.now, when)
            return item
        assert timeout is not None
        self.now += timeout
        raise Empty

    def get_nowait(self):
        return self.get(timeout=0.0)

    def sleep(self, duration):
        self.now += duration


def test_wall_clock_rate_latest_selection_and_final_sample():
    queue = ScheduledQueue([(0.0, observation(0).ground_truth),
                            (0.05, observation(1).ground_truth),
                            (0.10, observation(2).ground_truth),
                            (0.15, observation(3).ground_truth),
                            (0.25, observation(4).ground_truth), (0.26, None)])
    sent = []
    stats = send_loop(queue, SenderSettings(), lambda data: sent.append((queue.now, json.loads(data))),
                      clock=lambda: queue.now, sleep=queue.sleep)
    assert [message['sample']['header']['step_index'] for _, message in sent] == [0, 3, 4]
    assert [when for when, _ in sent] == pytest.approx([0.0, TEST_INTERVAL_S, 2 * TEST_INTERVAL_S])
    assert stats == {'sent': 3, 'skipped': 2, 'errors': 0, 'error': None}


def test_late_worker_has_no_catchup_burst_or_duplicate():
    queue = ScheduledQueue([(0.0, observation(0).ground_truth),
                            (1.0, observation(1).ground_truth),
                            (1.01, observation(2).ground_truth), (1.02, None)])
    times = []
    stats = send_loop(queue, SenderSettings(), lambda data: times.append(queue.now),
                      clock=lambda: queue.now, sleep=queue.sleep)
    assert times == pytest.approx([0.0, 1.0, 1.0 + TEST_INTERVAL_S])
    assert stats['sent'] == len(times) == 3


def test_worker_reports_encode_or_send_failure():
    queue = ScheduledQueue([(0.0, observation().ground_truth)])
    stats = send_loop(queue, SenderSettings(), Mock(side_effect=OSError('send failed')),
                      clock=lambda: queue.now, sleep=queue.sleep)
    assert stats['sent'] == 0 and stats['errors'] == 1
    assert 'send failed' in stats['error']
    bad = replace(observation().ground_truth, position_m=(float('nan'), 0.0, 0.0))
    queue = ScheduledQueue([(0.0, bad)])
    stats = send_loop(queue, SenderSettings(), Mock(), clock=lambda: queue.now, sleep=queue.sleep)
    assert stats['errors'] == 1


def test_overflow_is_nonblocking_and_separately_counted():
    sender = TelemetrySender(SenderSettings(queue_capacity=1))
    sender._queue = Mock()
    sender._queue.put_nowait.side_effect = Full
    sender._active = True
    sender._poll = Mock()
    sender.consume(observation())
    assert sender.statistics.overflow_drops == 1
    assert sender.statistics.submitted == sender.statistics.skipped == 0
    sender._queue.put_nowait.assert_called_once()


def test_spawned_sender_datagrams_and_shutdown():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(('127.0.0.1', 0))
        receiver.settimeout(TEST_WAIT_S)
        sender = TelemetrySender(SenderSettings(port=receiver.getsockname()[1]))
        try:
            sender.start()
            assert sender._process.is_alive()
            sender.consume(observation())
            first = json.loads(receiver.recv(65535))
            for step in range(1, TEST_STEPS + 1):
                sender.consume(observation(step))
            sender.close()
            messages = [first]
            while messages[-1]['sample']['header']['step_index'] != TEST_STEPS:
                messages.append(json.loads(receiver.recv(65535)))
            assert messages[0]['sample']['header']['step_index'] == 0
            for message in messages:
                Draft202012Validator(SCHEMA).validate(message['sample'])
                assert message['timestamp'] == message['sample']['header']['simulation_time_s']
            stats = sender.statistics
            assert stats.submitted == TEST_STEPS + 1
            assert stats.sent + stats.skipped == stats.submitted
            assert stats.unsent == stats.errors == stats.overflow_drops == 0
            assert not any(child.name == 'plotjuggler-udp-sender' for child in multiprocessing.active_children())
            sender.close()
        finally:
            sender.close()


def test_dead_sender_disables_streaming_without_affecting_caller():
    sender = TelemetrySender(SenderSettings())
    try:
        sender.start()
        sender._process.terminate()
        sender._process.join(timeout=TEST_WAIT_S)
        sender.consume(observation())
        sender.consume(observation(1))
        assert sender.statistics.errors == 1
        assert sender.statistics.error and sender.statistics.submitted == 0
    finally:
        sender.close()


def test_slow_final_send_has_bounded_shutdown():
    settings = SenderSettings(send_rate_hz=0.01, shutdown_timeout_s=0.1)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(('127.0.0.1', 0))
        receiver.settimeout(TEST_WAIT_S)
        sender = TelemetrySender(replace(settings, port=receiver.getsockname()[1]))
        try:
            sender.start()
            sender.consume(observation())
            receiver.recv(65535)
            sender.consume(observation(1))
            started = time.monotonic()
            sender.close()
            assert time.monotonic() - started < settings.shutdown_timeout_s + TEST_CLOSE_MARGIN_S
            assert sender.statistics.errors == 1
            assert sender.statistics.unsent >= 1
            assert not any(child.name == 'plotjuggler-udp-sender' for child in multiprocessing.active_children())
        finally:
            sender.close()


@pytest.mark.parametrize('failure,expected', [(RuntimeError('flight failed'), 1), (KeyboardInterrupt(), 130)])
def test_cli_closes_sender_when_flight_fails(monkeypatch, failure, expected):
    from examples.position_square import main, demo
    sender = Mock()
    monkeypatch.setattr('tools.zmq_render.publisher.RenderPublisher', Mock(return_value=Mock()))
    monkeypatch.setattr('tools.zmq_render.control.SimulationControl', Mock(return_value=Mock()))
    monkeypatch.setattr('tools.telemetry_sender.sender.TelemetrySender', Mock(return_value=sender))
    run = Mock(side_effect=failure)
    monkeypatch.setattr(demo, 'run_demo', run)
    assert main.main(['--headless', '--telemetry-config', str(CONFIG_PATH)]) == expected
    sender.start.assert_called_once()
    assert run.call_count == 1 and run.call_args.kwargs['gui'] is False
    observation = object()
    run.call_args.kwargs['on_observation'](observation)
    sender.consume.assert_called_once_with(observation)
    sender.close.assert_called_once()


def test_invalid_sender_settings_do_not_start_physics(tmp_path, monkeypatch):
    from examples.position_square import main, demo
    path = tmp_path / 'bad.yaml'
    path.write_text('send_rate_hz: false')
    run = Mock()
    monkeypatch.setattr(demo, 'run_demo', run)
    assert main.main(['--telemetry-config', str(path)]) == 2
    run.assert_not_called()


def test_callback_exception_closes_physics(monkeypatch):
    from examples.position_square import demo
    instances = []
    original = demo.RcSimulation
    def factory(*args):
        instance = original(*args)
        instances.append(instance)
        return instance
    monkeypatch.setattr(demo, 'RcSimulation', factory)
    with pytest.raises(RuntimeError, match='callback failed'):
        demo.run_demo(gui=False, on_observation=Mock(side_effect=RuntimeError('callback failed')))
    assert instances[0]._environment._physics._client is None


def test_callback_includes_reset_every_step_and_final():
    from examples.position_square import demo
    observed = []
    result = demo.run_demo(gui=False, on_observation=lambda obs: observed.append(obs.ground_truth))
    assert observed[0].step_index == 0
    assert observed[-1] == result.observation.ground_truth
    assert [state.step_index for state in observed] == list(range(len(observed)))


def test_spawn_failure_is_reported_and_resources_close(monkeypatch):
    sender = TelemetrySender(SenderSettings())
    process = Mock()
    process.start.side_effect = OSError('spawn failed')
    process.pid = None
    monkeypatch.setattr(sender._context, 'Process', Mock(return_value=process))
    try:
        sender.start()
        sender.consume(observation())
        assert sender.statistics.errors == 1
        assert 'spawn failed' in sender.statistics.error
    finally:
        sender.close()


def test_layout_and_tasks_reference_wire_fields():
    import xml.etree.ElementTree as ET
    root = ET.parse('plotjuggler/layouts/ground_truth_position.xml').getroot()
    plots = root.findall('.//plot')
    assert [plot.attrib['mode'] for plot in plots] == ['XYPlot', 'TimeSeries']
    xy = plots[0].find('curve')
    assert xy.attrib['x_topic'] == xy.attrib['y_topic'] == 'zmq/data'
    assert xy.attrib['x_field'] == 'position_m[0]'
    assert xy.attrib['y_field'] == 'position_m[1]'
    assert plots[1].find('curve').attrib['field'] == 'position_m[2]'
    assert root.find('left_panel_state').attrib['streaming_source'] == 'ZMQ Subscriber'
    layout_dir = Path('plotjuggler/layouts')
    assert (layout_dir / root.find('.//fileInfo').attrib['filename']).is_file()
    tasks = json.loads(Path('.vscode/tasks.json').read_text())['tasks']
    viewer = next(task for task in tasks if task['label'] == 'Open PlotJuggler ZMQ ground truth layout')
    flight = next(task for task in tasks if task['label'] == 'Run position square with ZeroMQ renderer')
    assert '--layout' in viewer['args']
    assert '--render-config' in flight['args']


def test_shutdown_retries_final_rejected_state():
    sender = TelemetrySender(SenderSettings(queue_capacity=1))
    queue = sender._queue = Mock()
    sender._queue.put_nowait.side_effect = Full
    sender._active = True
    sender._poll = Mock()
    final = observation(TEST_STEPS)
    sender.consume(final)
    sender.close()
    assert sender._submitted == 1 and sender.statistics.overflow_drops == 1
    assert [call.args[0] for call in queue.put.call_args_list] == [final.ground_truth, None]
    assert all(0 <= call.kwargs['timeout'] <= sender.settings.shutdown_timeout_s
               for call in queue.put.call_args_list)
