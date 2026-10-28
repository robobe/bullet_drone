from dataclasses import replace
import struct
import socket

import pytest

from bullet_drone.betaflight.backend import BetaflightBackend, RACER_TO_BETAFLIGHT, unpack_motor
from bullet_drone.betaflight.fdm import FDM_FORMAT, pack_fdm
from bullet_drone.betaflight.bridge import UdpSitlBridge
from bullet_drone.models import BarometerReading, DroneObservation, DroneState, ImuReading, SensorReadings


def observation(step=0, episode='episode'):
    state = DroneState(episode, step, step / 240, (0, 0, 1), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
    return DroneObservation(state, SensorReadings(
        ImuReading(episode, step, step / 240, (1, 2, 3), (4, 5, 6)),
        BarometerReading(episode, step, step / 240, 100000, 1)))


def test_fdm_contains_sensor_axes_pressure_and_no_truth_attitude():
    values = struct.unpack(FDM_FORMAT, pack_fdm(observation()))
    assert values[1:7] == (4, -5, -6, -1, -2, -3)
    assert values[7:11] == (1, 0, 0, 0)
    assert values[-1] == 100000
    bad = replace(observation(), sensors=SensorReadings())
    with pytest.raises(ValueError):
        pack_fdm(bad)


class Bridge:
    def __init__(self):
        self.packets = []
        self.motor = []

    def open(self): pass
    def send_fdm(self, packet): self.packets.append(packet)
    def receive_motor_packets(self):
        packets, self.motor = self.motor, []
        return packets
    def close(self): pass


class Vehicle:
    max_speed_rad_s = 2000


def test_betaflight_quad_x_motor_order_matches_racer_positions():
    expected_racer_indices = (2, 3, 1, 0)

    assert tuple(RACER_TO_BETAFLIGHT.index(index) for index in range(4)) == expected_racer_indices


def test_backend_maps_fresh_normalized_motors_and_stops_when_stale():
    now = [10.]
    bridge = Bridge()
    backend = BetaflightBackend(Vehicle(), bridge, clock=lambda: now[0])
    backend.reset(observation())
    bridge.motor = [(struct.pack('<4f', .1, .2, .3, .4), 10.)]
    command = backend.command(observation(0))
    assert command.speeds_rad_s == pytest.approx((800, 600, 200, 400))
    now[0] += .101
    assert backend.command(observation(1)).speeds_rad_s == (0, 0, 0, 0)


@pytest.mark.parametrize('packet', [b'', struct.pack('<4f', 0, 0, 0, 1.01), struct.pack('<4f', 0, 0, 0, float('nan'))])
def test_invalid_motor_packet(packet):
    with pytest.raises(ValueError):
        unpack_motor(packet)


def test_udp_bridge_sends_fdm_and_receives_motor_packet_on_loopback():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as fdm:
        fdm.bind(('127.0.0.1', 0))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(('127.0.0.1', 0))
            motor_port = probe.getsockname()[1]
        bridge = UdpSitlBridge(motor_port=motor_port, fdm_port=fdm.getsockname()[1], clock=lambda: 3.)
        bridge.open()
        try:
            bridge.send_fdm(b'fdm')
            assert fdm.recvfrom(16)[0] == b'fdm'
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.sendto(b'motors', ('127.0.0.1', bridge.motor_port))
            assert bridge.receive_motor_packets() == [(b'motors', 3.)]
        finally:
            bridge.close()
