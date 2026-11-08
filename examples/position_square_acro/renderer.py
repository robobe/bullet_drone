"""Own the exported Godot renderer for this example."""

from contextlib import ExitStack
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import tempfile
import time
from urllib.parse import urlsplit
from uuid import uuid4

import zmq
from bullet_simulation_protocol import load_transport
from bullet_image_transport.shm import FrameReader, SLOTS, slot_name, topic_prefix

from tools.zmq_render.publisher import RenderPublisher


IMAGE_TOPIC = 'camera/front/image'
STARTUP_TIMEOUT_S = 10.0
SHUTDOWN_TIMEOUT_S = 3.0
POLL_TIMEOUT_MS = 50
NANOSECONDS_PER_MICROSECOND = 1000
ERROR_LOG_CHARACTERS = 4000
SHM_DIRECTORY = Path('/dev/shm')
SHM_LOG_PATTERN = re.compile(r'^SHM publisher session: ([0-9a-f]{32})\r?$', re.MULTILINE)


class ExternalRenderer:
    def __init__(self, executable, config):
        self.executable = Path(executable).resolve()
        if not self.executable.is_file() or not os.access(self.executable, os.X_OK):
            raise FileNotFoundError(
                f'Renderer missing or not executable: {self.executable}. '
                'Run: uv run --locked python external_render/build.py')
        self.publisher = RenderPublisher(config, profile_name='position_square_acro',
                                         backend='betaflight')
        self.transport = load_transport(self.publisher.transport_path)
        self.physics_id = uuid4().hex
        self.process = self.log = self.images = self.reader = None
        self.owned_sessions = set()
        self.resources = ExitStack()

    def __enter__(self):
        try:
            for name in ('image_endpoint', 'control_endpoint'):
                address = urlsplit(self.transport[name])
                with socket.socket() as probe:
                    try:
                        probe.bind((address.hostname, address.port))
                    except OSError as error:
                        raise RuntimeError(f'Renderer port unavailable: {self.transport[name]}') from error
            self.log = self.resources.enter_context(tempfile.TemporaryFile(mode='w+'))
            self._start_zmq()
            self._start_godot()
            return self
        except BaseException:
            self.resources.close()
            raise

    def _start_zmq(self):
        context = self.resources.enter_context(zmq.Context())
        self.images = self.resources.enter_context(context.socket(zmq.SUB))
        self.images.setsockopt(zmq.LINGER, 0)
        self.images.setsockopt(zmq.RCVHWM, 1)
        self.images.setsockopt(zmq.SUBSCRIBE, topic_prefix(IMAGE_TOPIC))
        self.images.connect(self.transport['image_endpoint'])
        self.reader = FrameReader(IMAGE_TOPIC)
        self.resources.callback(self.reader.close)
        self.resources.callback(self.publisher.close)
        self.publisher.start()

    def _start_godot(self):
        self.process = subprocess.Popen(
            [str(self.executable), '--disable-vsync', '--', '--transport-config',
             str(self.publisher.transport_path.resolve())],
            stdout=self.log, stderr=subprocess.STDOUT)
        self.resources.callback(self._stop_process)

    def ensure_running(self):
        if self.publisher.error:
            raise RuntimeError(self.publisher.error)
        if self.process is None or self.process.poll() is not None:
            detail = ''
            if self.log is not None:
                self.log.seek(0)
                detail = self.log.read()[-ERROR_LOG_CHARACTERS:]
            raise RuntimeError(f'External renderer exited before mission completion.\n{detail}')

    def set_state(self, observation, state, reason):
        source = observation.ground_truth
        self.publisher.set_state(dict(
            schema_version=1, physics_id=self.physics_id, controller_id=None,
            episode_id=source.episode_id, state=state, reason=reason,
            simulation_time_s=source.simulation_time_s))

    def consume(self, observation):
        self.ensure_running()
        state = observation.ground_truth
        source = dict(
            physics_id=self.physics_id, episode_id=state.episode_id,
            stream_id=self.publisher.stream, step_index=state.step_index,
            simulation_time_s=state.simulation_time_s,
            measurement_us=time.monotonic_ns() // NANOSECONDS_PER_MICROSECOND)
        self.publisher.consume(observation, source=source)

    def wait_ready(self, observation, *, timeout_s=STARTUP_TIMEOUT_S):
        self.set_state(observation, 'stopped', 'renderer_startup')
        self.consume(observation)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.ensure_running()
            if self.images.poll(POLL_TIMEOUT_MS):
                frame = self.reader.read(self.images.recv())
                if frame is not None:
                    metadata, _ = frame
                    source = metadata.get('source', {})
                    if (source.get('stream_id') == self.publisher.stream
                            and source.get('episode_id') == observation.ground_truth.episode_id):
                        self.owned_sessions.add(metadata['session'])
                        return frame
        raise TimeoutError(f'No rendered camera image within {timeout_s:g} seconds')

    def _stop_process(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=SHUTDOWN_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=SHUTDOWN_TIMEOUT_S)
        if self.log is not None:
            self.log.seek(0)
            self.owned_sessions.update(SHM_LOG_PATTERN.findall(self.log.read()))
        for session in self.owned_sessions:
            for index in range(SLOTS):
                (SHM_DIRECTORY / slot_name(session, index).lstrip('/')).unlink(missing_ok=True)

    def __exit__(self, *exc):
        return self.resources.__exit__(*exc)
