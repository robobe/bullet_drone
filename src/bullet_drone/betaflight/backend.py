"""Observation-to-rotor adapter with injected transport, encoder and clock."""
from math import isfinite
import struct
import time

from bullet_drone.commands import RotorCommand
from bullet_drone.contracts import LifecycleError
from .fdm import pack_fdm

MOTOR_FORMAT = '<4f'
MOTOR_PACKET_BYTES = struct.calcsize(MOTOR_FORMAT)
MOTOR_TIMEOUT_S = .1
RACER_TO_BETAFLIGHT = (3, 2, 0, 1)
ZERO_OUTPUTS = (0.,) * 4


def unpack_motor(packet):
    if len(packet) != MOTOR_PACKET_BYTES:
        raise ValueError('Expected four normalized motor outputs')
    values = struct.unpack(MOTOR_FORMAT, packet)
    if any(not isfinite(value) or not 0 <= value <= 1 for value in values):
        raise ValueError('Motor outputs must be finite and in [0,1]')
    return values


class BetaflightBackend:
    allocation = None

    def __init__(self, vehicle, bridge, *, encoder=pack_fdm, clock=time.monotonic):
        self.bridge, self.encoder, self.clock = bridge, encoder, clock
        self.max_speed = vehicle.max_speed_rad_s
        self.episode = None
        self.next_step = None
        self.latest = ZERO_OUTPUTS
        self.received_at = None
        self.closed = False

    def reset(self, observation):
        if self.closed:
            raise LifecycleError('Betaflight adapter is closed')
        packet = self.encoder(observation)
        self.bridge.open()
        self.latest, self.received_at = ZERO_OUTPUTS, None
        self.episode = observation.ground_truth.episode_id
        self.next_step = observation.ground_truth.step_index + 1
        self.bridge.send_fdm(packet)

    def command(self, observation):
        state = observation.ground_truth
        if self.closed or state.episode_id != self.episode or state.step_index + 1 != self.next_step:
            raise LifecycleError('Stale observation or uninitialized Betaflight adapter')
        self.bridge.send_fdm(self.encoder(observation))
        for packet, received_at in self.bridge.receive_motor_packets():
            try:
                values = unpack_motor(packet)
            except ValueError:
                continue
            self.latest, self.received_at = values, received_at
        fresh = self.received_at is not None and 0 <= self.clock() - self.received_at <= MOTOR_TIMEOUT_S
        outputs = self.latest if fresh else ZERO_OUTPUTS
        speeds = tuple(outputs[index] * self.max_speed for index in RACER_TO_BETAFLIGHT)
        self.next_step += 1
        return RotorCommand(speeds, state.episode_id, state.step_index + 1)

    def close(self):
        self.closed = True
        self.episode = None
        self.bridge.close()
