"""Read Betaflight MAVLink attitude and barometer telemetry in a worker."""
from dataclasses import dataclass
from queue import Empty, Full, Queue
from threading import Event, Thread
import time
from math import isfinite

from pymavlink import mavutil
from bullet_drone.geometry import from_rpy
from bullet_drone.models import ControlFeedback

ENDPOINT = 'tcp:127.0.0.1:5762'
MAX_AGE_S = 0.5


@dataclass(frozen=True)
class SensorSample:
    altitude_m: float
    vertical_speed_m_s: float
    attitude: ControlFeedback
    received_at: float


class SensorWorker:
    def __init__(self, endpoint=ENDPOINT):
        self.endpoint = endpoint
        self.samples = Queue(maxsize=1)
        self.stop = Event()
        self.error = None
        self.thread = Thread(target=self._run, name='red-truck-mavlink', daemon=True)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self.stop.set()
        self.thread.join()

    def latest(self, required=True):
        if self.error:
            raise RuntimeError(f'MAVLink sensor reader failed: {self.error}') from self.error
        try:
            sample = self.samples.get_nowait()
            self.cached = sample
        except Empty:
            sample = getattr(self, 'cached', None)
        if sample is None or time.monotonic() - sample.received_at > MAX_AGE_S:
            if not required:
                return None
            raise RuntimeError('MAVLink attitude/barometer feedback missing or stale')
        return sample

    def _run(self):
        connection = None
        previous = None
        attitude = None
        attitude_time = 0.0
        altitude_time = 0.0
        speed = 0.0
        try:
            connection = mavutil.mavlink_connection(self.endpoint, autoreconnect=False)
            while not self.stop.is_set():
                message = connection.recv_match(type=['ATTITUDE', 'VFR_HUD'], blocking=True, timeout=.1)
                if message is None:
                    continue
                now = time.monotonic()
                if message.get_type() == 'ATTITUDE':
                    rpy = (message.roll, -message.pitch, message.yaw)
                    if not all(isfinite(v) for v in rpy):
                        raise ValueError('Nonfinite MAVLink attitude')
                    attitude = ControlFeedback(from_rpy(rpy), (0.0, 0.0, 0.0))
                    attitude_time = now
                else:
                    altitude = float(message.alt)
                    if not isfinite(altitude):
                        raise ValueError('Nonfinite MAVLink altitude')
                    if previous is not None and now > altitude_time:
                        dt = now - altitude_time
                        speed += dt / (.2 + dt) * ((altitude - previous) / dt - speed)
                    previous = altitude
                    altitude_time = now
                if attitude is not None and previous is not None:
                    self._publish(SensorSample(previous, speed, attitude,
                                               min(attitude_time, altitude_time)))
        except Exception as error:
            self.error = error
        finally:
            if connection is not None:
                connection.close()

    def _publish(self, sample):
        try:
            self.samples.put_nowait(sample)
        except Full:
            try:
                self.samples.get_nowait()
            except Empty:
                pass
            self.samples.put_nowait(sample)
