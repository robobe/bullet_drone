"""Thread-owned ZeroMQ transports; callers exchange bounded immutable snapshots."""
from concurrent.futures import Future
from contextlib import ExitStack
from copy import deepcopy
import json
from math import isfinite
from queue import Queue, Empty, Full
from threading import Thread, Event, Lock
import time
from uuid import uuid4

import zmq

QUEUE_CAPACITY = 16
POLL_MS = 5
START_TIMEOUT_S = 5
STOP_TIMEOUT_S = 5
MAX_MESSAGE_BYTES = 64 * 1024 * 1024


def encode(value):
    return json.dumps(value.to_dict() if hasattr(value, 'to_dict') else value,
                      allow_nan=False, separators=(',', ':'),
                      default=lambda message: message.to_dict()).encode()


def decode(raw):
    def invalid(value):
        raise ValueError(f'invalid JSON constant {value}')
    value = json.loads(raw.decode(), parse_constant=invalid)
    if not isinstance(value, dict):
        raise ValueError('expected JSON object')
    return value


class ZmqIo:
    """Optional PUB, SUB, REQ and REP sockets, all owned by one worker.

    SUB supports an exact multipart topic or a single-frame image descriptor
    whose bytes begin with subscription_prefix. No handlers run in this thread.
    """
    def __init__(self, *, publish_endpoint=None, subscribe_endpoint=None,
                 topics=(), subscription_prefix=None, request_endpoint=None,
                 reply_endpoint=None):
        self.endpoints = publish_endpoint, subscribe_endpoint, request_endpoint, reply_endpoint
        self.topics = tuple(topics)
        self.prefix = subscription_prefix
        self.outgoing = {}
        self.incoming = Queue(QUEUE_CAPACITY)
        self.requests = Queue(QUEUE_CAPACITY)
        self.replies = Queue(QUEUE_CAPACITY)
        self.clients = Queue(QUEUE_CAPACITY)
        self.lock = Lock()
        self.stop = Event()
        self.ready = Event()
        self.error = None
        self.thread = None

    def start(self):
        if self.thread is not None:
            raise RuntimeError('transport already started')
        self.thread = Thread(target=self._run, name='zmq-io', daemon=True)
        self.thread.start()
        if not self.ready.wait(START_TIMEOUT_S) or self.error:
            self.close()
            raise RuntimeError(self.error or 'ZeroMQ startup timeout')
        return self

    def publish(self, topic, value):
        if not self.endpoints[0] or self.stop.is_set():
            raise RuntimeError('publisher is not available')
        # Each topic has one replaceable snapshot; callers use a fixed topic set.
        snapshot = value if hasattr(value, 'to_dict') else deepcopy(value)
        with self.lock:
            if topic not in self.outgoing and len(self.outgoing) >= QUEUE_CAPACITY:
                raise RuntimeError('publication topic capacity exceeded')
            self.outgoing[topic] = snapshot

    @staticmethod
    def _take(queue):
        try:
            return queue.get_nowait()
        except Empty:
            return None

    def receive(self):
        return self._take(self.incoming)

    def poll_request(self):
        return self._take(self.requests)

    def reply(self, token, result):
        if not self.endpoints[3]:
            raise RuntimeError('reply endpoint is not configured')
        self.replies.put_nowait((token, deepcopy(result)))

    def request(self, value, timeout_s=1.):
        if not self.endpoints[2]:
            raise RuntimeError('request endpoint is not configured')
        if type(timeout_s) not in (int, float) or not isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('timeout must be positive')
        future = Future()
        self.clients.put_nowait((deepcopy(value), timeout_s, future))
        return future

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(STOP_TIMEOUT_S)
            if self.thread.is_alive():
                raise RuntimeError('ZeroMQ worker did not stop')

    def __enter__(self):
        return self.start()

    def __exit__(self, *args):
        self.close()

    def _run(self):
        sockets = []
        active = pending = None
        try:
            with ExitStack() as resources:
                context = zmq.Context()
                resources.callback(context.term)
                poller = zmq.Poller()

                def socket(kind, endpoint, bind, *, transient=False):
                    result = context.socket(kind)
                    if not transient:
                        sockets.append(result)
                        resources.callback(result.close)
                    result.setsockopt(zmq.LINGER, 0)
                    result.setsockopt(zmq.SNDTIMEO, POLL_MS)
                    result.setsockopt(zmq.SNDHWM, QUEUE_CAPACITY)
                    result.setsockopt(zmq.RCVHWM, QUEUE_CAPACITY)
                    result.setsockopt(zmq.MAXMSGSIZE, MAX_MESSAGE_BYTES)
                    (result.bind if bind else result.connect)(endpoint)
                    return result

                pub_ep, sub_ep, req_ep, rep_ep = self.endpoints
                pub = socket(zmq.PUB, pub_ep, True) if pub_ep else None
                sub = socket(zmq.SUB, sub_ep, False) if sub_ep else None
                rep = socket(zmq.REP, rep_ep, True) if rep_ep else None
                req = None
                if sub:
                    for topic in self.topics:
                        sub.setsockopt(zmq.SUBSCRIBE, topic)
                    if self.prefix is not None:
                        sub.setsockopt(zmq.SUBSCRIBE, self.prefix)
                    poller.register(sub, zmq.POLLIN)
                if rep:
                    poller.register(rep, zmq.POLLIN)
                self.ready.set()
                try:
                    while not self.stop.is_set():
                        with self.lock:
                            snapshots, self.outgoing = self.outgoing, {}
                        if pub:
                            for topic, value in snapshots.items():
                                try:
                                    pub.send_multipart([topic, encode(value)], zmq.DONTWAIT)
                                except zmq.Again:
                                    pass
                        if req_ep and active is None:
                            item = self._take(self.clients)
                            if item:
                                value, timeout, future = item
                                client_token = uuid4().hex
                                req = socket(zmq.REQ, req_ep, False, transient=True)
                                req.send(encode(value), zmq.DONTWAIT)
                                poller.register(req, zmq.POLLIN)
                                active = client_token, time.monotonic()+timeout, future
                        if sub is None and rep is None and active is None:
                            # An empty ZeroMQ poller returns immediately instead of waiting.
                            self.stop.wait(POLL_MS / 1000)
                            events = {}
                        else:
                            events = dict(poller.poll(POLL_MS))
                        if sub in events:
                            parts = sub.recv_multipart()
                            accepted = ((len(parts) == 2 and parts[0] in self.topics) or
                                        (len(parts) == 1 and self.prefix is not None and parts[0].startswith(self.prefix)))
                            if accepted:
                                try:
                                    self.incoming.put_nowait(parts)
                                except Full:
                                    self._take(self.incoming)
                                    self.incoming.put_nowait(parts)
                        if rep in events and pending is None:
                            try:
                                parts = rep.recv_multipart()
                                if len(parts) != 1:
                                    raise ValueError('expected one request frame')
                                message = decode(parts[0])
                                token = uuid4().hex
                                self.requests.put_nowait(dict(token=token, received_at=time.monotonic(), payload=message))
                                pending = token
                            except (ValueError, UnicodeError, RecursionError, Full):
                                rep.send(encode(dict(success=False, reason='invalid_or_busy_request')))
                        if pending is not None:
                            result = self._take(self.replies)
                            if result and result[0] == pending:
                                rep.send(encode(result[1]))
                                pending = None
                        if active is not None:
                            client_token, deadline, future = active
                            finished = False
                            if req in events:
                                try:
                                    response = decode(req.recv())
                                    if not future.done():
                                        future.set_result(response)
                                except (ValueError, UnicodeError, RecursionError) as error:
                                    if not future.done():
                                        future.set_exception(error)
                                finished = True
                            elif time.monotonic() >= deadline:
                                if not future.done():
                                    future.set_exception(TimeoutError('request outcome unknown'))
                                finished = True
                            if finished:
                                poller.unregister(req)
                                req.close()
                                active = None
                finally:
                    if req is not None:
                        req.close()
                    for item in sockets:
                        item.close()
                    if active and not active[2].done():
                        active[2].set_exception(RuntimeError('transport closed'))
                    while (item := self._take(self.clients)) is not None:
                        item[2].set_exception(RuntimeError('transport closed'))
        except Exception as error:
            self.error = str(error)
        finally:
            self.ready.set()


class ImageSubscription:
    """Latest queued single-frame image descriptor; worker owns its SUB socket."""
    def __init__(self, endpoint, prefix):
        self.io = ZmqIo(subscribe_endpoint=endpoint, subscription_prefix=prefix).start()
        self.pending = None

    def poll(self, timeout_ms=0):
        if self.io.error:
            raise RuntimeError(self.io.error)
        deadline = time.monotonic() + timeout_ms / 1000
        while self.pending is None:
            self.pending = self.io.receive()
            if self.pending is not None or time.monotonic() >= deadline:
                break
            time.sleep(min(POLL_MS / 1000, max(0, deadline-time.monotonic())))
        return self.pending is not None

    def recv(self):
        if self.pending is None:
            raise RuntimeError('poll before receiving')
        parts, self.pending = self.pending, None
        return parts[0]

    def close(self):
        self.io.close()
