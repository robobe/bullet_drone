"""Physics capture scheduling; transport worker owns serialization and ZeroMQ I/O."""
from dataclasses import asdict
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
from bullet_drone.rendering.config import CameraSettings, number


POSE_SCHEMA_VERSION = 2
RENDER_DRONE_ID = 'drone_1'
QUEUE_CAPACITY = 1
START_TIMEOUT_S = 15
STOP_TIMEOUT_S = 5
DEFAULT_PUBLISH_RATES_HZ = dict(camera_info=1., profile=1., state=1., wind=10.)
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
        self.control_state = None
        self.wind_state = None
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

    def set_state(self, state):
        from bullet_simulation_protocol.control import validate_state
        validate_state(state)
        with self.snapshot_lock:
            self.control_state = dict(state)

    def set_wind(self, state):
        from bullet_simulation_protocol.wind import validate_state
        validate_state(state)
        with self.snapshot_lock:
            self.wind_state = state

    def consume(self, observation, *, source=None):
        if self.thread is None or self.stop.is_set():
            raise RuntimeError('publisher is not running')
        if self.error:
            return  # Failure is logged by the worker; physics keeps running.
        state = resolve_pose(observation, self.pose_sources)
        with self.snapshot_lock:
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
                next_state = 0.0
                next_profile = 0.0
                next_wind = 0.0
                next_idle_pose = 0.0
                wire_sequence = 0
                previous_state = None
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
                    from bullet_simulation_protocol.control import STATE_TOPIC
                    with self.snapshot_lock:
                        control_state = self.control_state
                        latest = self.latest_pose
                        source = self.latest_source
                        wind_state = self.wind_state
                    now = time.monotonic()
                    if wind_state is not None and now >= next_wind:
                        from bullet_simulation_protocol.wind import STATE_TOPIC as WIND_TOPIC
                        io.publish(WIND_TOPIC, wind_state)
                        next_wind = now + 1 / self.rates['wind']
                    changed = control_state != previous_state
                    if control_state is not None and (changed or now >= next_state):
                        io.publish(STATE_TOPIC, control_state)
                        next_state = now + 1 / self.rates['state']
                        previous_state = control_state
                    try:
                        _, state, source = self.queue.get(timeout=WORKER_POLL_S)
                    except Empty:
                        if control_state is None or control_state['state'] == 'running' or latest is None or (not changed and now < next_idle_pose):
                            continue
                        state = latest
                        next_idle_pose = now + 1 / self.rates['state']
                    sequence = wire_sequence
                    wire_sequence += 1
                    packet = {'schema_version': POSE_SCHEMA_VERSION, 'stream_id': self.stream,
                              'sequence': sequence, 'publisher_drops': self.dropped,
                              **{name: value for name, value in asdict(state).items()
                                 if name in ('drone_id', 'episode_id', 'step_index',
                                             'simulation_time_s', 'position_m', 'orientation_xyzw',
                                             'linear_velocity_m_s')}}
                    # The renderer has one fixed scene slot; retain the simulation ID elsewhere.
                    packet['drone_id'] = RENDER_DRONE_ID
                    if source is not None:
                        packet.update(schema_version=3, source=source)
                    io.publish(b'render/drone_1/pose', packet)
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
