import copy
from pathlib import Path

import pytest
import yaml

from bullet_drone.config import ConfigurationError, DroneSettings, YamlConfigLoader
from bullet_drone.vehicle import load_vehicle_parameters


def valid_data():
    return {
        "simulation": {"timestep_s": 1 / 240, "max_steps": 240, "gravity_m_s2": [0, 0, -9.81]},
        "drone": {"mass_kg": 1, "initial_position_m": [0, 0, 1]},
    }


def write_config(tmp_path, data):
    path = tmp_path / "simulation.yaml"
    path.write_text(yaml.safe_dump(data))
    return path


def test_config_loader(tmp_path):
    path = write_config(tmp_path, valid_data())
    config = YamlConfigLoader().load(path)
    assert config.simulation.gui is False
    assert config.drone.initial_position_m == (0.0, 0.0, 1.0)
    assert config.drone.drone_id == 'drone_1'


def test_configured_drone_id(tmp_path):
    data = valid_data()
    data['drone']['drone_id'] = 'survey-drone-2'
    assert YamlConfigLoader().load(write_config(tmp_path, data)).drone.drone_id == 'survey-drone-2'
    assert DroneSettings(1, (0,0,1)).drone_id == 'drone_1'


def test_heavy_quad_urdf_selection(tmp_path):
    data = valid_data()
    data['drone'].update(urdf='heavy_quad', mass_kg=5.)
    config = YamlConfigLoader().load(write_config(tmp_path, data))
    vehicle = load_vehicle_parameters(config.drone.urdf)
    assert config.drone.urdf == 'heavy_quad'
    assert vehicle.rotor_positions_m[0] == pytest.approx((.240416, .240416, 0.))
    assert vehicle.max_thrust_per_rotor_n == pytest.approx(30.65625)
    assert vehicle.max_thrust_per_rotor_n > 5. * 9.81 / 4


def test_unknown_urdf_rejected():
    with pytest.raises(ConfigurationError, match='drone.urdf'):
        DroneSettings(1, (0, 0, 1), urdf='unknown')


@pytest.mark.parametrize('profile', ['internal_heavy_quad.yaml', 'betaflight_heavy_quad.yaml'])
def test_workspace_heavy_quad_profiles_keep_render_identity(profile):
    config = YamlConfigLoader().load(Path(__file__).resolve().parents[2] / 'config' / profile)
    assert config.drone.drone_id == 'drone_1'
    assert config.drone.urdf == 'heavy_quad'


@pytest.mark.parametrize('drone_id', ['', ' \t\n', None, True, 2, [], 'a' * 129])
def test_invalid_drone_id_rejected(tmp_path, drone_id):
    data = valid_data()
    data['drone']['drone_id'] = drone_id
    with pytest.raises(ConfigurationError, match='drone.drone_id'):
        YamlConfigLoader().load(write_config(tmp_path, data))
    with pytest.raises(ConfigurationError, match='drone.drone_id'):
        DroneSettings(1, (0,0,1), drone_id=drone_id)


def test_drone_id_length_boundary(tmp_path):
    data = valid_data()
    data['drone']['drone_id'] = 'a' * 128
    assert YamlConfigLoader().load(write_config(tmp_path, data)).drone.drone_id == 'a' * 128


@pytest.mark.parametrize("section,key,value", [
    ("simulation", "timestep_s", 0),
    ("simulation", "timestep_s", -1),
    ("simulation", "timestep_s", True),
    ("simulation", "timestep_s", float("nan")),
    ("simulation", "timestep_s", float("inf")),
    ("simulation", "max_steps", 0),
    ("simulation", "max_steps", True),
    ("simulation", "max_steps", 1.5),
    ("simulation", "gui", "false"),
    ("simulation", "gravity_m_s2", [0, 0]),
    ("simulation", "gravity_m_s2", [0, False, -9.81]),
    ("simulation", "gravity_m_s2", [0, 0, float("inf")]),
    ("drone", "mass_kg", -1),
    ("drone", "mass_kg", "1"),
    ("drone", "initial_position_m", "0, 0, 1"),
    ("drone", "initial_position_m", [0, 0, float("nan")]),
    ("drone", "unknown", 1),
])
def test_invalid_values(tmp_path, section, key, value):
    data = valid_data()
    data[section][key] = value
    with pytest.raises(ConfigurationError):
        YamlConfigLoader().load(write_config(tmp_path, data))


@pytest.mark.parametrize("data", [None, [], {"simulation": []}, {**valid_data(), "extra": 1}])
def test_invalid_root_or_missing_fields(tmp_path, data):
    with pytest.raises(ConfigurationError):
        YamlConfigLoader().load(write_config(tmp_path, data))


def test_missing_and_malformed_yaml(tmp_path):
    data = copy.deepcopy(valid_data())
    del data["drone"]["mass_kg"]
    with pytest.raises(ConfigurationError, match="missing"):
        YamlConfigLoader().load(write_config(tmp_path, data))
    path = tmp_path / "bad.yaml"
    path.write_text("simulation: [")
    with pytest.raises(ConfigurationError, match="Cannot read YAML"):
        YamlConfigLoader().load(path)
    with pytest.raises(ConfigurationError, match="Cannot read YAML"):
        YamlConfigLoader().load(tmp_path / "absent.yaml")


def test_simulation_config_rejects_legacy_flight_scenario(tmp_path):
    data = valid_data()
    assert not hasattr(YamlConfigLoader().load(write_config(tmp_path, data)), "flight")
    data["flight"] = {"mode": "hover"}
    with pytest.raises(ConfigurationError, match="unknown keys: flight"):
        YamlConfigLoader().load(write_config(tmp_path, data))
