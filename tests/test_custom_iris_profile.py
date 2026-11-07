from dataclasses import asdict
from pathlib import Path

import pybullet as p
import pytest
import yaml

from bullet_drone.config import YamlConfigLoader
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.vehicle import load_vehicle_parameters


CUSTOM_PROFILE = Path(__file__).resolve().parents[1] / 'custom/iris.yaml'


def _assert_all_fields_are_explicit(defaults, values, path='configuration'):
    assert isinstance(values, dict), f'{path} must be present as a mapping'
    for name, default in defaults.items():
        field_path = f'{path}.{name}'
        assert name in values, f'{field_path} is missing from the profile'
        if isinstance(default, dict):
            _assert_all_fields_are_explicit(default, values[name], field_path)


def test_iris_profile_explicitly_sets_every_configuration_field():
    config = YamlConfigLoader().load(CUSTOM_PROFILE)
    values = yaml.safe_load(CUSTOM_PROFILE.read_text(encoding='utf-8'))
    _assert_all_fields_are_explicit(asdict(config), values)


def test_iris_profile_loads_custom_urdf_and_vehicle_parameters():
    config = YamlConfigLoader().load(CUSTOM_PROFILE)
    vehicle = load_vehicle_parameters(config.drone.urdf)

    assert config.drone.urdf == str(CUSTOM_PROFILE.with_name('iris.urdf').resolve())
    assert config.drone.mass_kg == pytest.approx(1.5)
    expected_rotor_positions = (
        (0.13, -0.22, 0.023), (0.13, 0.22, 0.023),
        (-0.13, 0.20, 0.023), (-0.13, -0.20, 0.023),
    )
    for actual, expected in zip(vehicle.rotor_positions_m, expected_rotor_positions):
        assert actual == pytest.approx(expected)
    assert vehicle.reaction_torque_signs == (1, -1, 1, -1)
    assert vehicle.thrust_coefficient == pytest.approx(5.84e-6)
    assert vehicle.torque_coefficient == pytest.approx(5.84e-6 * 0.06)
    assert vehicle.max_thrust_per_rotor_n == pytest.approx(7.0664)
    assert vehicle.max_speed_rad_s == pytest.approx(1100.0)


def test_iris_profile_resets_pybullet_with_external_urdf():
    config = YamlConfigLoader().load(CUSTOM_PROFILE)
    physics = PyBulletPhysics(config)
    try:
        state = physics.reset()
        mass, *_ = p.getDynamicsInfo(physics._body, -1, physicsClientId=physics._client)
        assert state.position_m == config.drone.initial_position_m
        assert mass == pytest.approx(1.5)
        assert p.getNumJoints(physics._body, physicsClientId=physics._client) == 4
    finally:
        physics.close()
