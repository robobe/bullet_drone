"""Own one exported renderer process and its local pose/image transport."""
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

from bullet_simulation_protocol.transport import ImageSubscription
from bullet_simulation_protocol import load_transport
from bullet_image_transport.shm import FrameReader, SLOTS, slot_name, topic_prefix

from tools.zmq_render.publisher import RenderPublisher
from bullet_simulation_protocol.pose import PoseMessage


IMAGE_TOPIC = 'camera/front/image'
STARTUP_TIMEOUT_S = 10.0
SHUTDOWN_TIMEOUT_S = 3.0
POLL_TIMEOUT_MS = 50
NANOSECONDS_PER_MICROSECOND = 1000
ERROR_LOG_CHARACTERS = 4000
SHM_DIRECTORY = Path('/dev/shm')
SHM_LOG_PATTERN = re.compile(r'^SHM publisher session: ([0-9a-f]{32})\r?$', re.MULTILINE)


class ExternalRenderer:
    def __init__(self, executable, config, *, profile_name='external_render', backend='internal'):
        self.executable = Path(executable).resolve()
        if not self.executable.is_file() or not os.access(self.executable, os.X_OK):
            raise FileNotFoundError(
                f'Renderer missing or not executable: {self.executable}. '
                'Run: uv run --locked python external_render/build.py')
        self.publisher = RenderPublisher(config, profile_name=profile_name, backend=backend)
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
                    # Match ZeroMQ reuse semantics for recently closed TCP connections.
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
        self.images = ImageSubscription(self.transport['image_endpoint'], topic_prefix(IMAGE_TOPIC))
        self.resources.callback(self.images.close)
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

    def set_running(self, running):
        self.publisher.set_running(running)

    def consume(self, observation):
        self.ensure_running()
        self.publisher.consume(observation)

    def wait_ready(self, observation, *, timeout_s=STARTUP_TIMEOUT_S):
        self.set_running(False)
        self.consume(observation)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.ensure_running()
            if self.images.poll(POLL_TIMEOUT_MS):
                frame = self.reader.read(self.images.recv())
                if frame is not None:
                    metadata, _ = frame
                    expected = PoseMessage.from_state(observation.ground_truth)
                    camera = self.publisher.config['camera']
                    output = os.pread(self.log.fileno(), os.fstat(self.log.fileno()).st_size, 0).decode(errors='replace')
                    sessions = set(SHM_LOG_PATTERN.findall(output))
                    if (metadata['session'] in sessions and metadata.get('header') == expected.header.to_dict()
                            and metadata.get('drone_id') == expected.drone_id
                            and metadata['width'] == camera['width_px'] and metadata['height'] == camera['height_px']):
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
        # Signals bypass Godot destructors, so unlink only this child's named slots.
        for session in self.owned_sessions:
            for index in range(SLOTS):
                (SHM_DIRECTORY / slot_name(session, index).lstrip('/')).unlink(missing_ok=True)

    def __exit__(self, *exc):
        return self.resources.__exit__(*exc)
