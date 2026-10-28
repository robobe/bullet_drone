"""Check the Betaflight motor packet to headless PyBullet rotor mapping."""
import struct
from types import SimpleNamespace

from bullet_drone.betaflight.backend import BetaflightBackend, RACER_TO_BETAFLIGHT
from bullet_drone.config import DroneSettings, ForcesSettings, SimulationConfig, SimulationSettings
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.vehicle import load_racer_parameters

MOTOR_COUNT = 4
MOTOR_OUTPUT = 1.0
PHYSICS_TIMESTEP_S = 1 / 240
MAX_STEPS = 1
GRAVITY_M_S2 = (0.0, 0.0, -9.81)
INITIAL_POSITION_M = (0.0, 0.0, 1.0)
MASS_KG = 1.0
MIN_ANGULAR_RESPONSE_RAD_S = 1e-6
BETAFLIGHT_MOTOR_NAMES = ('M1', 'M2', 'M3', 'M4')
RACER_PROP_NAMES = ('front-left', 'rear-left', 'rear-right', 'front-right')
EXPECTED_RACER_INDICES = (2, 3, 1, 0)
EXPECTED_ROLL_PITCH_SIGNS = ((-1, 1), (-1, -1), (1, 1), (1, -1))


class Bridge:
    def __init__(self, packet):
        self.packet = packet

    def open(self):
        pass

    def send_fdm(self, packet):
        pass

    def receive_motor_packets(self):
        packet, self.packet = self.packet, None
        return [] if packet is None else [(packet, 0.0)]

    def close(self):
        pass


def sign(value):
    if abs(value) < MIN_ANGULAR_RESPONSE_RAD_S:
        return 0
    return 1 if value > 0 else -1


def motor_packet(index):
    outputs = [0.0] * MOTOR_COUNT
    outputs[index] = MOTOR_OUTPUT
    return struct.pack('<4f', *outputs)


def check_motor(index):
    vehicle = load_racer_parameters()
    physics = PyBulletPhysics(SimulationConfig(
        SimulationSettings(PHYSICS_TIMESTEP_S, MAX_STEPS, GRAVITY_M_S2),
        DroneSettings(MASS_KG, INITIAL_POSITION_M), forces=ForcesSettings.ideal()))
    try:
        initial = physics.reset()
        bridge = Bridge(motor_packet(index))
        backend = BetaflightBackend(vehicle, bridge, encoder=lambda observation: b'', clock=lambda: 0.0)
        backend.reset(SimpleNamespace(ground_truth=initial))
        command = backend.command(SimpleNamespace(ground_truth=initial))
        active = tuple(position for position, speed in enumerate(command.speeds_rad_s) if speed > 0)
        state = physics.step(command)
        roll, pitch, _ = state.angular_velocity_rad_s
        return active, (sign(roll), sign(pitch))
    finally:
        physics.close()


def main():
    failures = []
    print('motor  racer rotor   expected  measured  result')
    for index, name in enumerate(BETAFLIGHT_MOTOR_NAMES):
        active, measured = check_motor(index)
        expected_index = EXPECTED_RACER_INDICES[index]
        expected_signs = EXPECTED_ROLL_PITCH_SIGNS[index]
        passed = active == (expected_index,) and measured == expected_signs
        print(f'{name:<6} prop{expected_index} {RACER_PROP_NAMES[expected_index]:<11} '
              f'{expected_signs!s:<9} {measured!s:<9} {"PASS" if passed else "FAIL"}')
        if not passed:
            failures.append(name)
    if failures:
        raise SystemExit(f'Betaflight motor mapping failed: {", ".join(failures)}')


if __name__ == '__main__':
    main()
