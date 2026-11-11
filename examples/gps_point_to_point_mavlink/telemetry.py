"""Nonblocking firmware telemetry. No simulator observations enter this module."""
from dataclasses import dataclass
from math import isfinite
import time

from pymavlink import mavutil
from bullet_drone.geometry import from_rpy
from bullet_drone.models import ControlFeedback

ENDPOINT = 'tcp:127.0.0.1:5762'
FEEDBACK_MAX_AGE_S = 0.5
HEARTBEAT_MAX_AGE_S = 3.0
MIN_SATELLITES = 6
MAX_DRAIN_MESSAGES = 1000
GPS_DEGREES_SCALE = 1e7
GPS_ALTITUDE_MM_PER_M = 1000
VELOCITY_CM_PER_M = 100
REQUIRED = ('GPS_RAW_INT', 'GLOBAL_POSITION_INT', 'ATTITUDE', 'HEARTBEAT')


@dataclass(frozen=True)
class Feedback:
    position_m: tuple[float, float, float]
    velocity_m_s: tuple[float, float, float]
    attitude: ControlFeedback
    yaw_rad: float
    geographic_position: tuple[float, float, float]  # latitude deg, longitude deg, MSL altitude m


class Telemetry:
    def __init__(self, frame):
        self.frame = frame
        self.latest = {}
        self.connection = None

    def open(self):
        self.connection = mavutil.mavlink_connection(ENDPOINT)
        return self

    def poll(self):
        for _ in range(MAX_DRAIN_MESSAGES):
            message = self.connection.recv_match(blocking=False)
            if message is None:
                return
            if message.get_type() in REQUIRED:
                self.latest[message.get_type()] = (message, time.monotonic())
        raise RuntimeError('MAVLink receive backlog exceeded')

    def feedback(self, now):
        for kind in REQUIRED:
            if kind not in self.latest:
                raise RuntimeError(f'Missing MAVLink {kind}')
            age = now - self.latest[kind][1]
            limit = HEARTBEAT_MAX_AGE_S if kind == 'HEARTBEAT' else FEEDBACK_MAX_AGE_S
            if not 0 <= age <= limit:
                raise RuntimeError(f'Stale MAVLink {kind}')
        gps = self.latest['GPS_RAW_INT'][0]
        if gps.fix_type < 3 or gps.satellites_visible < MIN_SATELLITES:
            raise RuntimeError('MAVLink GPS fix unavailable')
        position = self.latest['GLOBAL_POSITION_INT'][0]
        attitude = self.latest['ATTITUDE'][0]
        # Pinned Gazebo/SITL telemetry matches world roll/yaw and negates world pitch.
        rpy = attitude.roll, -attitude.pitch, attitude.yaw
        if not all(isfinite(v) for v in rpy):
            raise RuntimeError('Invalid MAVLink attitude')
        # ATTITUDE rates are hardcoded zero in pinned firmware, only orientation is consumed.
        orientation = ControlFeedback(from_rpy(rpy), (0.0, 0.0, 0.0))
        geographic = (position.lat / GPS_DEGREES_SCALE, position.lon / GPS_DEGREES_SCALE,
                      position.alt / GPS_ALTITUDE_MM_PER_M)
        return Feedback(self.frame.local(*geographic),
                        (position.vy / VELOCITY_CM_PER_M, position.vx / VELOCITY_CM_PER_M, -position.vz / VELOCITY_CM_PER_M), orientation, rpy[2], geographic)

    def close(self):
        if self.connection is not None:
            self.connection.close()
