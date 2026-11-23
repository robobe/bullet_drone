"""Own the Godot viewer, image SHM reader and truck-contact subscription."""
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

from bullet_image_transport.shm import FrameReader, SLOTS, slot_name, topic_prefix
from bullet_simulation_protocol import load_transport
from bullet_simulation_protocol.scenario import CONTACT_TOPIC, validate_contact
from bullet_simulation_protocol.transport import ImageSubscription, ZmqIo
from bullet_simulation_protocol.pose import PoseMessage
from tools.zmq_render.publisher import RenderPublisher


IMAGE_TOPIC = 'camera/front/image'
MAX_FRAME_DESCRIPTORS_PER_POLL = 64
STARTUP_TIMEOUT_S = 10.0
SHUTDOWN_TIMEOUT_S = 3.0
ERROR_LOG_CHARACTERS = 4000
SHM_DIRECTORY = Path('/dev/shm')
SHM_LOG_PATTERN = re.compile(r'^SHM publisher session: ([0-9a-f]{32})\r?$', re.MULTILINE)


class ExternalRenderer:
    def __init__(self, executable, config):
        self.executable = Path(executable).resolve()
        if not self.executable.is_file() or not os.access(self.executable, os.X_OK):
            raise FileNotFoundError(f'Renderer missing or not executable: {self.executable}; build external_render first')
        self.publisher = RenderPublisher(config, profile_name='red_truck_visual', backend='betaflight')
        self.transport = load_transport(self.publisher.transport_path)
        self.process = self.log = self.images = self.reader = self.contacts = None
        self.owned_sessions = set()
        self.resources = ExitStack()

    def __enter__(self):
        try:
            for name in ('image_endpoint', 'control_endpoint'):
                address = urlsplit(self.transport[name])
                with socket.socket() as probe:
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    try:
                        probe.bind((address.hostname, address.port))
                    except OSError as error:
                        raise RuntimeError(f'Renderer port unavailable: {self.transport[name]}') from error
            self.log = self.resources.enter_context(tempfile.TemporaryFile(mode='w+'))
            self.publisher.start()
            self.resources.callback(self.publisher.close)
            self.images = ImageSubscription(self.transport['image_endpoint'], topic_prefix(IMAGE_TOPIC))
            self.resources.callback(self.images.close)
            self.reader = FrameReader(IMAGE_TOPIC)
            self.resources.callback(self.reader.close)
            self.contacts = ZmqIo(subscribe_endpoint=self.transport['control_endpoint'], topics=(CONTACT_TOPIC,)).start()
            self.resources.callback(self.contacts.close)
            self.process = subprocess.Popen(
                [str(self.executable), '--disable-vsync', '--', '--transport-config',
                 str(self.publisher.transport_path.resolve())], stdout=self.log, stderr=subprocess.STDOUT)
            self.resources.callback(self._stop_process)
            return self
        except BaseException:
            self.resources.close()
            raise

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

    def ensure_running(self):
        if self.contacts.error:
            raise RuntimeError(self.contacts.error)
        if self.publisher.error:
            raise RuntimeError(self.publisher.error)
        if self.process.poll() is not None:
            size = os.fstat(self.log.fileno()).st_size
            detail = os.pread(self.log.fileno(), size, 0).decode(errors='replace')[-ERROR_LOG_CHARACTERS:]
            raise RuntimeError(f'External renderer exited before mission completion.\n{detail}')

    def consume(self, observation):
        self.ensure_running()
        self.publisher.consume(observation)

    def wait_ready(self, observation, *, timeout_s=STARTUP_TIMEOUT_S):
        self.publisher.set_running(False)
        self.consume(observation)
        deadline = time.monotonic() + timeout_s
        expected = PoseMessage.from_state(observation.ground_truth)
        camera = self.publisher.config['camera']
        while time.monotonic() < deadline:
            self.ensure_running()
            frame = self.poll_frame(timeout_ms=50)
            if frame is None:
                continue
            metadata, _ = frame
            size = os.fstat(self.log.fileno()).st_size
            output = os.pread(self.log.fileno(), size, 0).decode(errors='replace')
            sessions = set(SHM_LOG_PATTERN.findall(output))
            if (metadata['session'] in sessions and metadata.get('header') == expected.header.to_dict()
                    and metadata.get('drone_id') == expected.drone_id
                    and metadata['width'] == camera['width_px'] and metadata['height'] == camera['height_px']):
                self.owned_sessions.add(metadata['session'])
                return
        raise TimeoutError(f'No rendered camera image within {timeout_s:g} seconds')

    def poll_frame(self, timeout_ms=0):
        raw = None
        for _ in range(MAX_FRAME_DESCRIPTORS_PER_POLL):
            if not self.images.poll(timeout_ms if raw is None else 0):
                break
            raw = self.images.recv()
        return None if raw is None else self.reader.read(raw)

    def poll_contact(self, *, stream_id, current_step, max_age_steps):
        if self.contacts.error:
            raise RuntimeError(self.contacts.error)
        matched = False
        while (parts := self.contacts.receive()) is not None:
            if len(parts) != 2 or parts[0] != CONTACT_TOPIC:
                continue
            from bullet_simulation_protocol.transport import decode
            try:
                event = validate_contact(decode(parts[1]))
            except (ValueError, UnicodeError, RecursionError):
                continue
            if (event['stream_id'] == stream_id and event['step_index'] <= current_step
                    and current_step - event['step_index'] <= max_age_steps):
                matched = True
        return matched

    def set_running(self, running):
        self.publisher.set_running(running)

    def __exit__(self, *exc):
        return self.resources.__exit__(*exc)
