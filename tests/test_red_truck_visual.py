from types import SimpleNamespace
import socket
import time

import pytest

from bullet_drone.controllers import AltitudeController
from bullet_drone.controllers.internal import AttitudeController
from bullet_drone.commands import RcInput
from bullet_drone.models import ControlFeedback
from examples.red_truck_visual.telemetry import SensorSample
from bullet_drone.rc import RcCodec
from bullet_drone.vehicle import load_racer_parameters
from examples.red_truck_visual.mission import (
    Phase, RedTruckMission,
)
from examples.red_truck_visual.controllers.altitude import (
    ALTITUDE_POSITION_PID, ALTITUDE_VELOCITY_PID,
)
from examples.red_truck_visual.controllers.attitude import MAX_BODY_RATES_RAD_S
from examples.red_truck_visual.vision import TargetOffset, detect_target


def frame(width=5, height=5, step=None, encoding='rgb8'):
    step = width * 3 + 2 if step is None else step
    return dict(width=width, height=height, step=step, encoding=encoding), bytearray(step * height)


@pytest.mark.parametrize('encoding,red', [('rgb8', (220, 30, 35)), ('bgr8', (35, 30, 220))])
def test_detector_returns_normalized_centroid_and_ignores_row_padding(encoding, red):
    metadata, pixels = frame(encoding=encoding)
    for x in (2, 3, 4):
        for y in (1, 2, 3):
            index = y * metadata['step'] + x * 3
            pixels[index:index + 3] = bytes(red)
    assert detect_target(metadata, pixels) == TargetOffset(0.4, 0.0, 9)


def test_detector_rejects_small_red_noise_and_bad_layout():
    metadata, pixels = frame()
    pixels[:3] = bytes((220, 20, 20))
    assert detect_target(metadata, pixels) is None
    with pytest.raises(ValueError, match='layout'):
        detect_target(metadata, pixels[:-1])


def test_vision_worker_polls_and_exposes_only_fresh_detection():
    from examples.red_truck_visual.vision import VisionWorker

    metadata, pixels = frame(5, 5)
    for x, y in ((1, 1), (2, 1), (3, 1), (1, 2), (2, 2), (3, 2), (1, 3), (2, 3)):
        index = y * metadata['step'] + x * 3
        pixels[index:index + 3] = bytes((220, 30, 35))
    metadata['header'] = {'fid': 10}

    class Viewer:
        def poll_frame(self, timeout_ms):
            time.sleep(timeout_ms / 1000)
            if hasattr(self, 'returned'):
                return None
            self.returned = True
            return metadata, pixels

    with VisionWorker(Viewer()) as vision:
        deadline = time.monotonic() + 1
        while vision.target(10, 2) is None and time.monotonic() < deadline:
            time.sleep(.001)
        assert vision.target(10, 2) == TargetOffset(-0.05, -0.05, 8)
        assert vision.target(13, 2) is None


class MissionFixture(RedTruckMission):
    def command(self, observation, status, dt_s, target=None, **kwargs):
        state = observation.ground_truth
        sensors = SensorSample(state.position_m[2], state.linear_velocity_m_s[2],
                               ControlFeedback(state.orientation_xyzw, (0, 0, 0)), time.monotonic())
        return super().command(state.simulation_time_s, status, dt_s, target,
                               sensors=sensors, **kwargs)


def mission():
    vehicle = load_racer_parameters()
    max_thrust = 4 * vehicle.max_thrust_per_rotor_n
    codec = RcCodec(max_thrust, MAX_BODY_RATES_RAD_S)
    value = MissionFixture(
        AltitudeController(1.0, 9.81, max_thrust, ALTITUDE_POSITION_PID, ALTITUDE_VELOCITY_PID),
        AttitudeController((4.0, 4.0, 4.0), MAX_BODY_RATES_RAD_S), codec, max_thrust)
    value.reset(0.0, SensorSample(30, 0, ControlFeedback((0, 0, 0, 1), (0, 0, 0)), time.monotonic()))
    value.phase = Phase.APPROACH
    value.approach_start_altitude_m = 30.0
    value.approach_started_s = 0.0
    return value, codec


def observation(altitude, time_s, velocity=(0.0, 0.0, 0.0)):
    state = SimpleNamespace(
        episode_id='episode', step_index=int(time_s * 240), simulation_time_s=time_s,
        position_m=(0.0, 0.0, altitude), orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        linear_velocity_m_s=velocity, angular_velocity_rad_s=(0.0, 0.0, 0.0),
    )
    return SimpleNamespace(ground_truth=state)


def status(armed=True):
    return SimpleNamespace(armed=armed, arming_disabled_flags=0)


def test_image_errors_drive_roll_pitch_and_vertical_thrust():
    a, codec = mission()
    positive = a.command(observation(30.0, 0.1), status(), 1.0, TargetOffset(.5, .5, 100))
    b, _ = mission()
    negative = b.command(observation(30.0, 0.1), status(), 1.0, TargetOffset(.5, -.5, 100))
    assert positive[0] > 1500
    assert positive[1] > negative[1]
    assert codec.decode(RcInput(positive), 'episode', 25).mode == 'acro'


def test_dy_guidance_is_reduced_at_20_m_and_full_at_40_m():
    low, _ = mission()
    low.command(observation(20.0, 0.1), status(), 1.0, TargetOffset(0, .5, 100))
    high, _ = mission()
    high.command(observation(40.0, 0.1), status(), 1.0, TargetOffset(0, .5, 100))

    assert low.control_targets[0] < high.control_targets[0]
    assert low.altitude.last_output.velocity_target_m_s == pytest.approx(-5.0)
    assert high.altitude.last_output.velocity_target_m_s == pytest.approx(-5.5)


def test_lost_target_reacquires_search_timeout_lands_and_contact_completes():
    value, _ = mission()
    channels = value.command(observation(20.0, 1.0), status(), 1 / 240, None)
    assert value.phase == Phase.REACQUIRE
    assert channels[0:2] == (1500, 1500)

    value.phase = Phase.SEARCH
    value.search_started_s = 0.0
    value.command(observation(30.0, 31.0), status(), 1 / 240, None)
    assert value.phase == Phase.LAND

    value, _ = mission()
    channels = value.command(observation(5.0, 3.0), status(), 1 / 240, None, contact=True)
    assert value.phase == Phase.COMPLETE
    assert channels[4] == 1000


def test_contact_uses_exact_topic_and_requires_active_stream_and_fresh_pose():
    from bullet_simulation_protocol.scenario import CONTACT_TOPIC
    from bullet_simulation_protocol.transport import ZmqIo
    from examples.red_truck_visual.renderer import ExternalRenderer

    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        endpoint = f'tcp://127.0.0.1:{probe.getsockname()[1]}'
    with ZmqIo(publish_endpoint=endpoint) as publisher, ZmqIo(
            subscribe_endpoint=endpoint, topics=(CONTACT_TOPIC,)) as subscriber:
        renderer = ExternalRenderer.__new__(ExternalRenderer)
        renderer.contacts = subscriber
        deadline = time.monotonic() + 3
        matched = False
        while time.monotonic() < deadline and not matched:
            publisher.publish(CONTACT_TOPIC, dict(schema_version=1, event_id='other-18-hit', stream_id='other',
                step_index=18, target_id='red_truck', drone_position_m=[50, 0, 2]))
            publisher.publish(CONTACT_TOPIC, dict(schema_version=1, event_id='active-10-hit', stream_id='active',
                step_index=10, target_id='red_truck', drone_position_m=[50, 0, 2]))
            publisher.publish(CONTACT_TOPIC, dict(schema_version=1, event_id='active-18-hit', stream_id='active',
                step_index=18, target_id='red_truck', drone_position_m=[50, 0, 2]))
            publisher.publish(CONTACT_TOPIC + b'/extra', dict())
            matched = renderer.poll_contact(stream_id='active', current_step=20, max_age_steps=5)
            time.sleep(0.01)
        assert matched


def test_camera_poll_copies_only_newest_descriptor_and_bounds_backlog():
    from unittest.mock import Mock
    from examples.red_truck_visual.renderer import ExternalRenderer, MAX_FRAME_DESCRIPTORS_PER_POLL

    viewer = ExternalRenderer.__new__(ExternalRenderer)
    viewer.images = Mock()
    viewer.images.poll.return_value = True
    viewer.images.recv.side_effect = iter(range(MAX_FRAME_DESCRIPTORS_PER_POLL))
    viewer.reader = Mock()
    viewer.poll_frame()
    viewer.reader.read.assert_called_once_with(MAX_FRAME_DESCRIPTORS_PER_POLL - 1)


def test_vision_queue_keeps_newest_result_and_stops_worker():
    from threading import Event
    from examples.red_truck_visual.vision import VisionWorker

    metadata, pixels = frame()
    finished = Event()

    class Viewer:
        fid = 0

        def poll_frame(self, timeout_ms):
            self.fid += 1
            if self.fid <= 3:
                return dict(metadata, header={'fid': self.fid}), pixels
            finished.set()
            time.sleep(timeout_ms / 1000)
            return None

    with VisionWorker(Viewer()) as vision:
        assert finished.wait(1)
        assert vision.results.qsize() == 1
        assert vision.target(3, 2) is None
        assert vision.latest == (3, None)
    assert not vision.thread.is_alive()


def test_status_poll_does_not_block_physics_caller():
    from threading import Event
    from examples.red_truck_visual.status import StatusWorker

    entered, release = Event(), Event()
    initial = object()

    class Commands:
        calls = 0

        def read_status(self):
            self.calls += 1
            if self.calls > 1:
                entered.set()
                assert release.wait(1)
            return initial

    with StatusWorker(Commands()) as worker:
        try:
            assert entered.wait(1)
            assert worker.read() is initial
        finally:
            release.set()
    assert not worker.thread.is_alive()



def test_diagnostics_rate_limits_errors_but_reports_state_changes(monkeypatch):
    from loguru import logger
    from examples.red_truck_visual import diagnostics

    value, _ = mission()
    obs = observation(30, .1)
    target = TargetOffset(.2, .3, 100)
    channels = value.command(obs, status(), 1 / 240, target)
    clock = [1.0]
    monkeypatch.setattr(diagnostics.time, 'monotonic', lambda: clock[0])
    messages = []
    sink = logger.add(lambda message: messages.append(str(message)))
    try:
        reporter = diagnostics.Diagnostics()
        reporter.report(value, obs.ground_truth, target, channels)
        value.transitions.append((Phase.REACQUIRE, .2))
        reporter.report(value, obs.ground_truth, None, channels)
        clock[0] += .11
        reporter.report(value, obs.ground_truth, None, channels)
    finally:
        logger.remove(sink)
    assert sum('VISION phase=' in message for message in messages) == 2
    assert any('REACQUIRE' in message and 'STATE' in message for message in messages)
    assert any('dx=+0.2000 dy=+0.3000' in message for message in messages)
    assert any('target=LOST' in message for message in messages)


def test_takeoff_speed_and_deceleration_preserve_other_phase_limits():
    value, _ = mission()
    value.phase = Phase.TAKEOFF
    value.command(observation(5, 1), status(), 1 / 240)
    assert value.altitude.last_output.velocity_target_m_s == 4
    value.command(observation(19.9, 2), status(), 1 / 240)
    assert 0 < value.altitude.last_output.velocity_target_m_s < .2
    value.phase = Phase.LAND
    value.command(observation(20, 3), status(), 1 / 240)
    assert value.altitude.last_output.velocity_target_m_s == -1.5



def test_packet_pacing_counts_work_and_does_not_catch_up(monkeypatch):
    from bullet_drone.betaflight.bridge import UdpSitlBridge
    from examples.red_truck_visual.timing import PacedSitlBridge

    clock = [0.0]
    sent = []
    monkeypatch.setattr(UdpSitlBridge, 'send_fdm', lambda self, packet: sent.append(clock[0]))

    def sleep(delay):
        clock[0] += delay

    bridge = PacedSitlBridge(.004, clock=lambda: clock[0], sleep=sleep)
    bridge.send_fdm(b'')
    clock[0] += .001  # processing consumes part of the interval
    bridge.send_fdm(b'')
    clock[0] += .020  # missed deadline: no backlog
    bridge.send_fdm(b'')
    bridge.send_fdm(b'')
    assert sent == pytest.approx([0, .004, .024, .028])
    assert bridge.sent == 4




def test_mavlink_attitude_cannot_refresh_stale_altitude(monkeypatch):
    from examples.red_truck_visual import telemetry
    worker = telemetry.SensorWorker()
    clock = [1.0]
    messages = iter([
        SimpleNamespace(get_type=lambda: 'ATTITUDE', roll=.1, pitch=.2, yaw=.3),
        SimpleNamespace(get_type=lambda: 'VFR_HUD', alt=12.0),
        SimpleNamespace(get_type=lambda: 'ATTITUDE', roll=.1, pitch=.2, yaw=.3),
    ])

    class Connection:
        calls = 0
        closed = False

        def recv_match(self, **kwargs):
            self.calls += 1
            if self.calls == 3:
                clock[0] = 2.0
                worker.stop.set()
            return next(messages)

        def close(self):
            self.closed = True

    connection = Connection()
    monkeypatch.setattr(telemetry.mavutil, 'mavlink_connection', lambda *a, **k: connection)
    monkeypatch.setattr(telemetry.time, 'monotonic', lambda: clock[0])
    worker._run()
    assert connection.closed
    assert worker.error is None
    with pytest.raises(RuntimeError, match='stale'):
        worker.latest()


def test_mission_requires_explicit_mavlink_sample():
    value, _ = mission()
    with pytest.raises(RuntimeError, match='MAVLink'):
        RedTruckMission.command(value, .1, status(), 1 / 240, TargetOffset(0, 0, 100))


def test_takeoff_allows_sensor_noise_before_los_acquisition():
    value, _ = mission()
    value.phase = Phase.TAKEOFF
    target = TargetOffset(.1, .2, 100)
    value.command(observation(19.3, 1, velocity=(0, 0, 1.0)), status(), 1 / 240, target)
    value.command(observation(19.5, 1.31, velocity=(0, 0, .8)), status(), 1 / 240, target)
    assert value.phase == Phase.SEARCH
    value.command(observation(29.5, 1.32), status(), 1 / 240, target)
    assert value.phase == Phase.APPROACH
