"""Physics capture scheduling; transport worker owns serialization and ZeroMQ I/O."""
from bullet_simulation_protocol.transport import ZmqIo
from bullet_simulation_protocol.camera_info import CameraInfo, CameraInfoMessage, CAMERA_INFO_TOPIC
import json
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Thread, Lock
from uuid import uuid4

from loguru import logger
from bullet_drone.config import ConfigurationError, PoseSources
from bullet_drone.pose import resolve_pose
from bullet_simulation_protocol.pose import PoseMessage, POSE_TOPIC
from bullet_simulation_protocol.flight import FEEDBACK_TOPIC
from tools.zmq_render.feedback import from_observation
from bullet_drone.rendering.config import CameraSettings, number


QUEUE_CAPACITY = 1
START_TIMEOUT_S = 15
STOP_TIMEOUT_S = 5
DEFAULT_PUBLISH_RATES_HZ = dict(camera_info=1., profile=1.)
IDLE_POSE_REFRESH_HZ = 1.
SCHEDULE_TOLERANCE_S = 1e-12
WORKER_POLL_S = 0.002


class RenderPublisher:
    def __init__(self, path, *, pose_sources: PoseSources = PoseSources(),
                 profile_name='unknown', drone_model='racer', backend='internal'):
        if not isinstance(pose_sources, PoseSources):
            raise ConfigurationError('pose_sources must be PoseSources')
        path = Path(path).resolve()
        try:
            config = json.loads(path.read_text())
            if set(config) - {'transport_config', 'camera', 'fps', 'publish_rates_hz'} or not {'transport_config', 'camera', 'fps'} <= set(config):
                raise ValueError('expected transport_config, camera and fps')
            camera = config['camera']
            if set(camera) != {'width_px', 'height_px', 'fov_x_deg'}:
                raise ValueError('expected camera width_px, height_px and fov_x_deg')
            CameraSettings(camera['width_px'], camera['height_px'], camera['fov_x_deg'])
            number(config['fps'], 'fps', 0)
            if config['fps'] > 240:
                raise ValueError('fps must not exceed the 240 Hz example physics rate')
            if camera['width_px'] * camera['height_px'] * 3 > 64 * 1024 * 1024:
                raise ValueError('image exceeds recorder 64 MiB limit')
            self.transport_path = path.parent / config['transport_config']
        except (OSError, ValueError, TypeError, KeyError) as exc:
            raise ConfigurationError(f'Invalid render configuration: {exc}') from exc
        self.rates = DEFAULT_PUBLISH_RATES_HZ.copy()
        configured_rates = config.get('publish_rates_hz', {})
        if not isinstance(configured_rates, dict) or set(configured_rates) - set(self.rates):
            raise ConfigurationError('invalid publication rates')
        self.rates.update(configured_rates)
        for rate in self.rates.values():
            number(rate, 'publication rate', 0)
            if rate <= 0:
                raise ConfigurationError('publication rate must be positive')
        self.pose_sources = pose_sources
        if not all(isinstance(value, str) and value for value in (profile_name, drone_model, backend)):
            raise ConfigurationError('render profile metadata must be nonblank strings')
        self.config = config
        self.stream = str(uuid4())
        self.profile = dict(schema_version=1, stream_id=self.stream,
                            profile_name=profile_name, backend=backend, drone_model=drone_model)
        self.queue = Queue(QUEUE_CAPACITY)
        self.stop = Event()
        self.ready = Event()
        self.error = None
        self.thread = None
        self.episode = None
        self.next_capture = 0.0
        self.snapshot_lock = Lock()
        self.latest_pose = None
        self.latest_observation = None
        self.observation_sequence = 0
        self.control_state = None
        self.latest_source = None
        self.sequence = 0
        self.dropped = 0

    def start(self):
        if self.thread is not None:
            raise RuntimeError('publisher already started')
        self.thread = Thread(target=self._run, name='zmq-render', daemon=True)
        self.thread.start()
        if not self.ready.wait(START_TIMEOUT_S):
            self.close()
            raise RuntimeError('ZeroMQ publisher startup timed out')
        if self.error:
            self.close()
            raise RuntimeError(self.error)

    def set_running(self, running):
        if type(running) is not bool:
            raise ValueError('running must be boolean')
        with self.snapshot_lock:
            self.control_state = {'state': 'running' if running else 'stopped'}

    def consume(self, observation, *, source=None):
        if self.thread is None or self.stop.is_set():
            raise RuntimeError('publisher is not running')
        if self.error:
            return  # Failure is logged by the worker; physics keeps running.
        state = resolve_pose(observation, self.pose_sources)
        with self.snapshot_lock:
            self.latest_observation = observation
            self.observation_sequence += 1
            self.latest_pose = state
            self.latest_source = source
        if state.episode_id != self.episode:
            self.episode = state.episode_id
            self.next_capture = 0.0
        if state.simulation_time_s + SCHEDULE_TOLERANCE_S < self.next_capture:
            return
        self.next_capture = (int((state.simulation_time_s + SCHEDULE_TOLERANCE_S) * self.config['fps']) + 1) / self.config['fps']
        item = (self.sequence, state, source)
        self.sequence += 1
        try:
            self.queue.put_nowait(item)
        except Full:
            try:
                self.queue.get_nowait()
                self.dropped += 1
            except Empty:
                pass
            try:
                self.queue.put_nowait(item)
            except Full:
                self.dropped += 1

    def _run(self):
        import time
        try:
            from bullet_simulation_protocol import load_transport
            transport = load_transport(self.transport_path)
            with ZmqIo(publish_endpoint=transport['telemetry_endpoint']) as io:
                config = CameraInfoMessage(self.stream, self.config['fps'], CameraInfo.from_config(self.config['camera']))
                next_config = 0.0
                next_profile = 0.0
                next_idle_pose = 0.0
                wire_sequence = 0
                previous_state = None
                feedback_sequence = -1
                self.ready.set()
                while not self.stop.is_set():
                    if io.error:
                        raise RuntimeError(io.error)
                    if time.monotonic() >= next_config:
                        io.publish(CAMERA_INFO_TOPIC, config)
                        next_config = time.monotonic() + 1 / self.rates['camera_info']
                    if time.monotonic() >= next_profile:
                        io.publish(b'render/drone_1/profile', self.profile)
                        next_profile = time.monotonic() + 1 / self.rates['profile']
                    with self.snapshot_lock:
                        control_state = self.control_state
                        latest = self.latest_pose
                        source = self.latest_source
                        observation = self.latest_observation
                        observation_sequence = self.observation_sequence
                    if observation is not None and observation_sequence != feedback_sequence:
                        io.publish(FEEDBACK_TOPIC, from_observation(observation))
                        feedback_sequence = observation_sequence
                    now = time.monotonic()
                    changed = control_state != previous_state
                    previous_state = control_state
                    try:
                        _, state, source = self.queue.get(timeout=WORKER_POLL_S)
                    except Empty:
                        if control_state is None or control_state['state'] == 'running' or latest is None or (not changed and now < next_idle_pose):
                            continue
                        state = latest
                        next_idle_pose = now + 1 / IDLE_POSE_REFRESH_HZ
                    sequence = wire_sequence
                    wire_sequence += 1
                    io.publish(POSE_TOPIC, PoseMessage.from_state(state))
        except Exception as exc:
            self.error = f'ZeroMQ render publisher failed: {exc}'
            logger.error(self.error)
        finally:
            self.ready.set()

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(STOP_TIMEOUT_S)
            if self.thread.is_alive():
                logger.error('ZeroMQ render worker did not stop before timeout')
        logger.info('Render publisher: scheduled={}, queue_drops={}', self.sequence, self.dropped)
