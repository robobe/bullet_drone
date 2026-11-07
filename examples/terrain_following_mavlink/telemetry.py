"""Read-only Betaflight MAVLink monitoring, independent of flight control."""
from threading import Event, Thread
import time

from loguru import logger
from pymavlink import mavutil

ENDPOINT = 'tcp:127.0.0.1:5762'
HEARTBEAT_TIMEOUT_S = 10.0
STALE_AFTER_S = 3.0
REPORT_PERIOD_S = 1.0
RECEIVE_TIMEOUT_S = 0.1
STOP_TIMEOUT_S = 2.0
MONITORED_MESSAGES = ('HEARTBEAT', 'ATTITUDE', 'VFR_HUD')


class MavlinkMonitor:
    def __init__(self):
        self.stop = Event()
        self.connection = None
        self.thread = None

    def __enter__(self):
        try:
            self.connection = mavutil.mavlink_connection(ENDPOINT)
            if self.connection.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S) is None:
                raise TimeoutError('MAVLink heartbeat startup timeout')
            self.thread = Thread(target=self._run, name='mavlink-monitor')
            self.thread.start()
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def _run(self):
        heartbeat_at = time.monotonic()
        latest = {}
        report_at = heartbeat_at
        try:
            while not self.stop.is_set():
                message = self.connection.recv_match(blocking=True, timeout=RECEIVE_TIMEOUT_S)
                now = time.monotonic()
                if message is not None and message.get_type() != 'BAD_DATA':
                    kind = message.get_type()
                    latest[kind] = (message.to_dict(), now)
                    if kind == 'HEARTBEAT':
                        heartbeat_at = now
                if now >= report_at:
                    age = now - heartbeat_at
                    logger.log('WARNING' if age > STALE_AFTER_S else 'INFO',
                               'MAVLink heartbeat_age={:.2f}s stale={} telemetry={}',
                               age, age > STALE_AFTER_S,
                               {kind: (dict(data=latest[kind][0],
                                            age_s=round(now - latest[kind][1], 2),
                                            stale=now - latest[kind][1] > STALE_AFTER_S)
                                       if kind in latest else 'MISSING')
                                for kind in MONITORED_MESSAGES})
                    report_at = now + REPORT_PERIOD_S
        except Exception as error:
            logger.warning('MAVLink monitoring unavailable: {}', error)

    def __exit__(self, *args):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(STOP_TIMEOUT_S)
        if self.connection is not None:
            self.connection.close()
