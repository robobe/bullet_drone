"""Minimal camera transform with simulation-step image correlation."""
from dataclasses import dataclass
from math import isfinite

from .feedback import TimeStamp
from .wind import DictionaryMessage

POSE_TOPIC = b'render/drone_1/pose'
MAX_WIRE_INTEGER = 2**53 - 1
MAX_POSITION_M = 1e9
QUATERNION_TOLERANCE = 0.002


@dataclass(frozen=True)
class PoseHeader(DictionaryMessage):
    fid: int
    time_stamp: TimeStamp

    def __post_init__(self):
        if type(self.fid) is not int or not 0 <= self.fid <= MAX_WIRE_INTEGER or not isinstance(self.time_stamp, TimeStamp):
            raise ValueError('invalid pose header')

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict) or set(value) != {'fid', 'time_stamp'}:
            raise ValueError('invalid pose header fields')
        return cls(value['fid'], TimeStamp.from_dict(value['time_stamp']))


@dataclass(frozen=True)
class PoseMessage(DictionaryMessage):
    header: PoseHeader
    drone_id: str
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]

    def __post_init__(self):
        if not isinstance(self.header, PoseHeader) or not isinstance(self.drone_id, str) or not self.drone_id.strip() or len(self.drone_id) > 128:
            raise ValueError('invalid pose identity')
        for vector, count, bound in ((self.position_m, 3, MAX_POSITION_M), (self.orientation_xyzw, 4, 1.)):
            if not isinstance(vector, tuple) or len(vector) != count or any(type(x) not in (int, float) or abs(x) > bound or not isfinite(x) for x in vector):
                raise ValueError('invalid pose vector')
        if abs(sum(x*x for x in self.orientation_xyzw)-1) > QUATERNION_TOLERANCE:
            raise ValueError('pose quaternion must be unit length')

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict) or set(value) != {'header', 'drone_id', 'position_m', 'orientation_xyzw'}:
            raise ValueError('invalid pose fields')
        if not isinstance(value['position_m'], (list, tuple)) or not isinstance(value['orientation_xyzw'], (list, tuple)):
            raise ValueError('invalid pose vectors')
        return cls(PoseHeader.from_dict(value['header']), value['drone_id'], tuple(value['position_m']), tuple(value['orientation_xyzw']))

    @classmethod
    def from_state(cls, state):
        return cls(PoseHeader(state.step_index, TimeStamp.from_seconds(state.simulation_time_s)),
                   state.drone_id, state.position_m, state.orientation_xyzw)
