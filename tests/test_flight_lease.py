import json
import time

import pytest
import zmq
from tools.zmq_render.flight import FlightLease, FlightReceiver
from bullet_msgs.flight import RC_TOPIC


def packet(**changes):
    value = dict(schema_version=1, physics_id='physics', episode_id='episode', app_id='app', sequence=1,
                 issued_us=1_000_000, valid_until_us=1_100_000, active=False,
                 channels=[1500, 1500, 1000, 1500, 1000, 2000] + [1000] * 12)
    value.update(changes)
    return value


def test_readiness_episode_owner_expiry_and_withdrawal():
    now = [1.]
    lease = FlightLease('physics', clock=lambda: now[0])
    assert not lease.ready()
    assert lease.accept(packet(), 'episode') and lease.ready()
    lease.begin('new_episode')
    assert lease.command() is None
    assert not lease.accept(packet(active=True), 'new_episode')
    value = packet(active=True, episode_id='new_episode')
    assert lease.accept(value, 'new_episode')
    assert lease.command() == value['channels']
    assert not lease.accept(value, 'new_episode')
    assert not lease.accept(dict(value, app_id='restarted', sequence=2), 'new_episode')
    assert lease.accept(dict(value, sequence=2, active=False), 'new_episode')
    assert lease.command() is None and lease.reason == 'app_withdrawn'
    assert lease.accept(dict(value, sequence=3), 'new_episode')
    assert lease.command() is None and lease.reason == 'app_withdrawn'
    lease.end()
    assert lease.accept(packet(), 'episode')
    lease.begin('new_episode')
    assert lease.accept(value, 'new_episode')
    now[0] = 1.1
    assert lease.command() is None and lease.reason == 'rc_expired'


def test_real_tcp_exact_topic_and_malformed_bounds():
    with zmq.Context() as context, context.socket(zmq.PUB) as pub:
        pub.setsockopt(zmq.LINGER, 0)
        port = pub.bind_to_random_port('tcp://127.0.0.1')
        lease = FlightLease('physics', clock=lambda: 1.)
        receiver = FlightReceiver(f'tcp://127.0.0.1:{port}', lease)
        try:
            time.sleep(.1)
            for topic, raw in ((RC_TOPIC + b'/extra', json.dumps(packet()).encode()), (RC_TOPIC, b'{')):
                pub.send_multipart([topic, raw])
            time.sleep(.02)
            receiver.poll('episode')
            assert not lease.ready()
            pub.send_multipart([RC_TOPIC, json.dumps(packet()).encode()])
            deadline = time.monotonic() + 1
            while not lease.ready() and time.monotonic() < deadline:
                receiver.poll('episode')
                time.sleep(.001)
            assert lease.ready()
        finally:
            receiver.close()
