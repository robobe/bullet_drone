"""Bounded local RC lease, owned and polled by the example's physics thread."""
import time

import zmq
from bullet_msgs.flight import RC_TOPIC, MAX_FLIGHT_BYTES, decode, validate_rc


POLL_LIMIT = 64
HWM = 4
ARM_INDEX = 4
THROTTLE_INDEX = 2
ARM_THRESHOLD = 1500
RC_MIN = 1000
MICROSECONDS_PER_SECOND = 1_000_000


class FlightLease:
    def __init__(self, physics_id, *, clock=time.monotonic):
        self.physics_id = physics_id
        self.clock = clock
        self.episode_id = None
        self.owner = None
        self.latest = None
        self.candidate = None
        self.sequence = 0
        self.reason = 'waiting_app'
        self.running = False
        self.withdrawn = False

    def accept(self, packet, episode_id):
        validate_rc(packet)
        now = int(self.clock() * MICROSECONDS_PER_SECOND)
        if (packet['physics_id'] != self.physics_id or packet['episode_id'] != episode_id
                or not packet['issued_us'] <= now < packet['valid_until_us']):
            return False
        if self.running:
            if packet['app_id'] != self.owner or packet['sequence'] <= self.sequence:
                return False
            self.sequence = packet['sequence']
            self.latest = packet
            self.withdrawn |= not packet['active']
        else:
            if packet['active'] or packet['channels'][ARM_INDEX] >= ARM_THRESHOLD or packet['channels'][THROTTLE_INDEX] != RC_MIN:
                return False
            if self.candidate and self.candidate['app_id'] == packet['app_id'] and packet['sequence'] <= self.candidate['sequence']:
                return False
            self.candidate = packet
        return True

    def ready(self):
        return self.candidate is not None and int(self.clock() * MICROSECONDS_PER_SECOND) < self.candidate['valid_until_us']

    def begin(self, episode_id):
        if not self.ready():
            raise ValueError('app is not ready')
        self.owner = self.candidate['app_id']
        self.episode_id = episode_id
        self.sequence = 0
        self.latest = None
        self.running = True
        self.withdrawn = False
        self.reason = 'waiting_first_command'

    def command(self):
        if not self.latest:
            return None
        now = int(self.clock() * MICROSECONDS_PER_SECOND)
        if now >= self.latest['valid_until_us']:
            self.reason = 'rc_expired'
            return None
        if self.withdrawn:
            self.reason = 'app_withdrawn'
            return None
        self.reason = 'active'
        return self.latest['channels']

    def end(self):
        self.running = False
        self.latest = self.candidate = None
        self.owner = None
        self.sequence = 0


class FlightReceiver:
    def __init__(self, endpoint, lease):
        self.lease = lease
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        try:
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.setsockopt(zmq.RCVHWM, HWM)
            self.socket.setsockopt(zmq.MAXMSGSIZE, MAX_FLIGHT_BYTES)
            self.socket.setsockopt(zmq.SUBSCRIBE, RC_TOPIC)
            self.socket.connect(endpoint)
        except BaseException:
            self.close()
            raise

    def poll(self, episode_id):
        for _ in range(POLL_LIMIT):
            try:
                parts = self.socket.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                break
            if len(parts) != 2 or parts[0] != RC_TOPIC:
                continue
            try:
                self.lease.accept(decode(parts[1], validate_rc), episode_id)
            except (ValueError, TypeError, UnicodeError):
                continue

    def close(self):
        self.socket.close(linger=0)
        self.context.term()
