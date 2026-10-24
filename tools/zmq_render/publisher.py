"""Physics capture scheduling; bounded worker owns serialization and ZeroMQ I/O."""
from dataclasses import asdict
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
TELEMETRY_HWM = 64
QUEUE_CAPACITY = 1
START_TIMEOUT_S = 15
STOP_TIMEOUT_S = 5
CONFIG_INTERVAL_S = 1
SCHEDULE_TOLERANCE_S = 1e-12
FLIGHT_FEEDBACK_HZ = 120
WORKER_POLL_S = 0.002
TERMINAL_INTERVAL_S = 0.1
ANALYSIS_QUEUE_CAPACITY = 256
GROUND_TRUTH_HZ = 100
MAX_ANALYSIS_BATCH = 16


class RenderPublisher:
    def __init__(self, path, *, pose_sources: PoseSources = PoseSources()):
        if not isinstance(pose_sources, PoseSources):
            raise ConfigurationError('pose_sources must be PoseSources')
        path = Path(path).resolve()
        try:
            config = json.loads(path.read_text())
            if set(config) != {'transport_config', 'camera', 'fps'}:
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
        self.pose_sources = pose_sources
        self.config = config
        self.stream = str(uuid4())
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
        self.spawn_state = None
        self.wind_state = None
        self.analysis_config = None
        self.analysis_queue = Queue(ANALYSIS_QUEUE_CAPACITY)
        self.analysis_drops = 0
        self.flight_config = self.feedback = self.latest_source = None
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
        from bullet_msgs.control import validate_state
        validate_state(state)
        with self.snapshot_lock:
            self.control_state = dict(state)

    def set_spawn(self, state):
        from bullet_msgs.spawn import validate_state
        validate_state(state)
        with self.snapshot_lock:
            self.spawn_state = state

    def set_wind(self, state):
        from bullet_msgs.wind import validate_state
        validate_state(state)
        with self.snapshot_lock:
            self.wind_state = state

    def set_flight(self, config, feedback):
        from bullet_msgs.flight import validate_config, validate_feedback
        validate_config(config)
        validate_feedback(feedback)
        with self.snapshot_lock:
            self.flight_config, self.feedback = config, feedback

    def set_analysis_config(self, settings, revisions):
        with self.snapshot_lock:
            self.analysis_config = dict(settings=settings, revisions=revisions)

    def consume_analysis(self, observation, source):
        # Immutable observations cross to the serializer; never block the physics loop.
        try:
            self.analysis_queue.put_nowait((observation, source))
        except Full:
            self.analysis_drops += 1

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
            import zmq
            from bullet_msgs import load_transport
            transport = load_transport(self.transport_path)
            with zmq.Context() as context, context.socket(zmq.PUB) as socket:
                socket.setsockopt(zmq.LINGER, 0)
                socket.setsockopt(zmq.SNDHWM, TELEMETRY_HWM)
                socket.bind(transport['telemetry_endpoint'])
                config = {'schema_version': 1, 'stream_id': self.stream,
                          'camera': self.config['camera'], 'fps': self.config['fps']}
                next_analysis_config = 0.0
                analysis_episode = None
                next_truth = 0.0
                next_config = 0.0
                next_state = 0.0
                next_spawn = 0.0
                next_wind = 0.0
                previous_spawn = None
                next_end = 0.0
                next_feedback = time.monotonic()
                next_idle_pose = 0.0
                wire_sequence = 0
                previous_state = None
                self.ready.set()
                while not self.stop.is_set():
                    if time.monotonic() >= next_config:
                        socket.send_multipart([b'render/drone_1/config', json.dumps(config).encode()], flags=zmq.DONTWAIT)
                        with self.snapshot_lock:
                            platform = self.flight_config
                        if platform is not None:
                            from bullet_msgs.flight import CONFIG_TOPIC
                            socket.send_multipart([CONFIG_TOPIC, json.dumps(platform).encode()], flags=zmq.DONTWAIT)
                        next_config = time.monotonic() + CONFIG_INTERVAL_S
                    from bullet_msgs.control import STATE_TOPIC
                    with self.snapshot_lock:
                        control_state = self.control_state
                        latest = self.latest_pose
                        source = self.latest_source
                        feedback = self.feedback
                        spawn_state = self.spawn_state
                        wind_state = self.wind_state
                        analysis_config = self.analysis_config
                    now = time.monotonic()
                    if analysis_config is not None and feedback is not None and now >= next_analysis_config:
                        packet = dict(schema_version=1, source=feedback['source'], role='physics', **analysis_config)
                        socket.send_multipart([b'analysis/physics/config', json.dumps(packet, allow_nan=False).encode()], flags=zmq.DONTWAIT)
                        next_analysis_config = now + CONFIG_INTERVAL_S
                    for _ in range(MAX_ANALYSIS_BATCH):
                        try:
                            observation, sample_source = self.analysis_queue.get_nowait()
                        except Empty:
                            break
                        if feedback is not None and sample_source['episode_id'] != feedback['source']['episode_id']:
                            continue
                        truth = observation.ground_truth
                        if truth.episode_id != analysis_episode:
                            analysis_episode, next_truth = truth.episode_id, 0.0
                        packet = dict(schema_version=1, source=sample_source, publisher_drops=self.analysis_drops)
                        sensors = dict(packet, sensors=asdict(observation.sensors),
                                       attitude_estimate=None if observation.attitude_estimate is None else asdict(observation.attitude_estimate))
                        socket.send_multipart([b'analysis/physics/sensors', json.dumps(sensors, allow_nan=False).encode()], flags=zmq.DONTWAIT)
                        if truth.simulation_time_s + SCHEDULE_TOLERANCE_S >= next_truth:
                            packet['ground_truth'] = asdict(truth)
                            socket.send_multipart([b'analysis/physics/ground_truth', json.dumps(packet, allow_nan=False).encode()], flags=zmq.DONTWAIT)
                            next_truth = (int((truth.simulation_time_s + SCHEDULE_TOLERANCE_S) * GROUND_TRUTH_HZ) + 1) / GROUND_TRUTH_HZ
                    if spawn_state is not None and (spawn_state != previous_spawn or now >= next_spawn):
                        from bullet_msgs.spawn import STATE_TOPIC as SPAWN_TOPIC, SPAWN_INTERVAL_S
                        socket.send_multipart([SPAWN_TOPIC, json.dumps(spawn_state).encode()], flags=zmq.DONTWAIT)
                        previous_spawn = spawn_state
                        next_spawn = now + SPAWN_INTERVAL_S
                    if wind_state is not None and now >= next_wind:
                        from bullet_msgs.wind import STATE_TOPIC as WIND_TOPIC, WIND_INTERVAL_S
                        socket.send_multipart([WIND_TOPIC, json.dumps(wind_state, allow_nan=False).encode()], flags=zmq.DONTWAIT)
                        next_wind = now + WIND_INTERVAL_S
                    if feedback is not None and now >= next_feedback:
                        from bullet_msgs.flight import FEEDBACK_TOPIC
                        socket.send_multipart([FEEDBACK_TOPIC, json.dumps(feedback).encode()], flags=zmq.DONTWAIT)
                        next_feedback += 1 / FLIGHT_FEEDBACK_HZ
                        if next_feedback < now:
                            next_feedback = now + 1 / FLIGHT_FEEDBACK_HZ
                    terminal = None if control_state is None else control_state.get('terminal_result')
                    if terminal is not None and now >= next_end:
                        from bullet_msgs.scenario import END_TOPIC
                        socket.send_multipart([END_TOPIC, json.dumps(terminal).encode()], flags=zmq.DONTWAIT)
                        next_end = now + TERMINAL_INTERVAL_S
                    changed = control_state != previous_state
                    if control_state is not None and (changed or now >= next_state):
                        socket.send_multipart([STATE_TOPIC, json.dumps(control_state).encode()], flags=zmq.DONTWAIT)
                        next_state = now + CONFIG_INTERVAL_S
                        previous_state = control_state
                    try:
                        _, state, source = self.queue.get(timeout=WORKER_POLL_S)
                    except Empty:
                        if control_state is None or control_state['state'] == 'running' or latest is None or (not changed and now < next_idle_pose):
                            continue
                        state = latest
                        next_idle_pose = now + CONFIG_INTERVAL_S
                    sequence = wire_sequence
                    wire_sequence += 1
                    packet = {'schema_version': POSE_SCHEMA_VERSION, 'stream_id': self.stream,
                              'sequence': sequence, 'publisher_drops': self.dropped,
                              **{name: value for name, value in asdict(state).items()
                                 if name in ('drone_id', 'episode_id', 'step_index',
                                             'simulation_time_s', 'position_m', 'orientation_xyzw',
                                             'linear_velocity_m_s')}}
                    if source is not None:
                        packet.update(schema_version=3, source=source)
                    socket.send_multipart([b'render/drone_1/pose', json.dumps(packet, allow_nan=False).encode()], flags=zmq.DONTWAIT)
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
