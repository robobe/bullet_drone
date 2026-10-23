"""Best-effort nonblocking datagrams. Viewer availability is never inferred."""
from dataclasses import asdict
import json
import socket
from uuid import uuid4
from loguru import logger
from bullet_drone.rendering.config import RenderSettings, identity
from bullet_drone.rendering.models import RenderSnapshot


def encode_packet(snapshot: RenderSnapshot, settings: RenderSettings, stream_id: str, sequence: int) -> bytes:
    identity(stream_id, 'stream_id')
    if type(sequence) is not int or sequence < 0:
        raise ValueError('sequence must be a nonnegative integer')
    if snapshot.drone_id != settings.drone_id:
        raise ValueError('snapshot drone_id must match configured publisher')
    return json.dumps(dict(version=1, stream_id=stream_id, sequence=sequence,
                           snapshot=asdict(snapshot), camera=asdict(settings.camera)),
                      allow_nan=False, separators=(',', ':')).encode('utf-8')


class UdpRenderer:
    def __init__(self, settings: RenderSettings):
        self.settings = settings
        self.stream_id = str(uuid4())
        self.sequence = 0
        self._socket = None
        self._closed = False
        self._warned = False
        self._address = None

    def reset(self, initial_snapshot: RenderSnapshot) -> None:
        if self._closed:
            raise RuntimeError('renderer is closed')
        if self._socket is None:
            try:
                self._address = socket.getaddrinfo(self.settings.udp.host, self.settings.udp.port,
                                                  socket.AF_INET, socket.SOCK_DGRAM)[0][4]
                self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._socket.setblocking(False)
            except OSError as exc:
                self._diagnostic(exc)
        self.render(initial_snapshot)

    def _diagnostic(self, exc):
        if not self._warned:
            logger.warning('Render UDP transport failed: {}', exc)
            self._warned = True

    def render(self, snapshot: RenderSnapshot) -> None:
        if self._closed:
            raise RuntimeError('renderer is closed')
        payload = encode_packet(snapshot, self.settings, self.stream_id, self.sequence)
        self.sequence += 1
        if self._socket is not None and self._address is not None:
            try:
                self._socket.sendto(payload, self._address)
            except OSError as exc:
                self._diagnostic(exc)

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            if self._socket is not None:
                self._socket.close()
