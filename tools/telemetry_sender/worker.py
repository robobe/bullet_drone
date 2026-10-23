"""Child-only conversion and UDP output; no physics or rendering imports."""

from dataclasses import asdict
import json
from queue import Empty
import signal
import socket
import time

from bullet_drone.telemetry.header import SCHEMA_VERSION
from bullet_drone.telemetry.topics.ground_truth import TOPIC


END_OF_STREAM = None
READY = 'ready'
FINISHED = 'finished'


def encode_datagram(state) -> bytes:
    data = asdict(state)
    header = {name: data.pop(name) for name in ('drone_id', 'episode_id', 'step_index', 'simulation_time_s')}
    header['schema_version'] = SCHEMA_VERSION
    return json.dumps({'timestamp': state.simulation_time_s, 'topic': TOPIC,
                       'sample': {'header': header, 'data': data}},
                      allow_nan=False, separators=(',', ':')).encode('utf-8')


def send_loop(queue, settings, send, *, clock=time.monotonic, sleep=time.sleep) -> dict:
    """Keep only the newest pending state and never catch up after a late send."""
    interval_s = 1 / settings.send_rate_hz
    next_send = clock()
    pending = None
    stopping = False
    stats = {'sent': 0, 'skipped': 0, 'errors': 0, 'error': None}
    try:
        while not stopping or pending is not None:
            if stopping:
                sleep(max(0.0, next_send - clock()))
            else:
                timeout = max(0.0, next_send - clock()) if pending is not None else None
                try:
                    item = queue.get(timeout=timeout)
                except Empty:
                    item = pending
                    pending = None
                if item is END_OF_STREAM:
                    stopping = True
                else:
                    if pending is not None:
                        stats['skipped'] += 1
                    pending = item
            if pending is None or clock() < next_send:
                continue
            # Limit the drain so an active producer cannot starve transmission.
            if not stopping:
                for _ in range(settings.queue_capacity):
                    try:
                        item = queue.get_nowait()
                    except Empty:
                        break
                    if item is END_OF_STREAM:
                        stopping = True
                        break
                    stats['skipped'] += 1
                    pending = item
            send(encode_datagram(pending))
            stats['sent'] += 1
            pending = None
            next_send = clock() + interval_s
    except Exception as exc:
        stats['errors'] += 1
        stats['error'] = f'{type(exc).__name__}: {exc}'
    return stats


def run_sender(queue, status, settings) -> None:
    """Spawn target. The status pipe carries startup and one terminal summary."""
    # The parent handles Ctrl+C and closes this process cooperatively.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setblocking(False)
            status.send((READY, None))
            stats = send_loop(queue, settings, lambda payload: sock.sendto(payload, (settings.host, settings.port)))
    except Exception as exc:
        stats = {'sent': 0, 'skipped': 0, 'errors': 1, 'error': f'{type(exc).__name__}: {exc}'}
    try:
        status.send((FINISHED, stats))
    except (BrokenPipeError, EOFError, OSError):
        pass  # Parent may already have closed after its shutdown deadline.
    finally:
        status.close()
        queue.close()
