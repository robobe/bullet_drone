import socket
import time
import pytest
import zmq
from bullet_simulation_protocol.transport import ZmqIo

TIMEOUT_S = 3


def endpoint():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0))
        return f'tcp://127.0.0.1:{sock.getsockname()[1]}'


def wait(call):
    deadline = time.monotonic()+TIMEOUT_S
    while time.monotonic() < deadline:
        result=call()
        if result is not None:
            return result
        time.sleep(.005)
    raise AssertionError('timed out')


def test_pubsub_exact_topic_latest_and_cleanup():
    address=endpoint()
    with ZmqIo(publish_endpoint=address) as pub, ZmqIo(subscribe_endpoint=address,topics=(b'pose',)) as sub:
        def exchange():
            pub.publish(b'pose/extra', {'bad':True})
            pub.publish(b'pose', {'step':42})
            return sub.receive()
        topic,payload=wait(exchange)
        assert topic==b'pose' and b'42' in payload
        assert pub.thread.is_alive()
    assert not pub.thread.is_alive() and not sub.thread.is_alive()


def test_request_reply_and_timeout_recovery():
    address=endpoint()
    with ZmqIo(reply_endpoint=address) as server,ZmqIo(request_endpoint=address) as client:
        future=client.request({'action':'test'})
        request=wait(server.poll_request)
        server.reply(request['token'],{'applied':True})
        assert future.result(TIMEOUT_S)=={'applied':True}
        future=client.request({'action':'slow'},timeout_s=.05)
        request=wait(server.poll_request)
        with pytest.raises(TimeoutError):future.result(TIMEOUT_S)
        server.reply(request['token'],{})
        future=client.request({'action':'again'})
        request=wait(server.poll_request)
        server.reply(request['token'],{'ok':True})
        assert future.result(TIMEOUT_S)['ok']


def test_malformed_request_gets_error_and_server_recovers():
    address=endpoint()
    with ZmqIo(reply_endpoint=address),zmq.Context() as ctx,ctx.socket(zmq.REQ) as req:
        req.connect(address)
        req.send(b'not json')
        assert req.poll(3000)
        assert req.recv_json()['success'] is False
        req.send(b'[' * 2000 + b']' * 2000)
        assert req.poll(3000)
        assert req.recv_json()['success'] is False


def test_bind_failure_closes_thread():
    address=endpoint()
    with ZmqIo(publish_endpoint=address):
        failed=ZmqIo(publish_endpoint=address)
        with pytest.raises(RuntimeError):failed.start()
        assert not failed.thread.is_alive()


def test_absent_consumers_do_not_block_snapshot_submission():
    with ZmqIo(publish_endpoint=endpoint()) as io:
        started = time.monotonic()
        for step in range(10000):
            io.publish(b'pose', {'step':step})
        assert time.monotonic()-started < TIMEOUT_S
        assert len(io.outgoing) <= 1
        assert io.error is None


@pytest.mark.parametrize('role', ['publish_endpoint', 'request_endpoint'])
def test_idle_worker_does_not_poll_an_empty_socket_set(monkeypatch, role):
    original = zmq.Poller.poll

    def checked_poll(self, timeout=None):
        assert self.sockets, 'empty poller would busy-spin'
        return original(self, timeout)

    monkeypatch.setattr(zmq.Poller, 'poll', checked_poll)
    with ZmqIo(**{role: endpoint()}) as io:
        time.sleep(.03)
        assert io.error is None
        assert io.thread.is_alive()
    assert not io.thread.is_alive()
