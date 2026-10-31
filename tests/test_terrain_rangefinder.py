from dataclasses import replace
import struct
import pytest

from bullet_drone.config import YamlConfigLoader
from bullet_drone.environment import DroneEnvironment
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.sensors import RangefinderSensor


def test_shared_terrain_returns_body_down_agl_in_centimetres():
    config = YamlConfigLoader().load("configs/basic.yaml")
    config = replace(config, simulation=replace(config.simulation, gui=False))
    physics = PyBulletPhysics(config)
    environment = DroneEnvironment(physics, rangefinder=RangefinderSensor(physics, config.sensors.rangefinder))
    try:
        reading = environment.reset().sensors.rangefinder
        assert reading.valid and reading.distance_cm == 100.0
    finally:
        environment.close()


def test_godot_bullet_frame_mapping_preserves_terrain_height():
    config = YamlConfigLoader().load("configs/heavy_quad.yaml")
    config = replace(config, simulation=replace(config.simulation, gui=False))
    heights = struct.unpack("<14641f", open("../terrain/terrain_heightmap.f32", "rb").read())
    physics = PyBulletPhysics(config)
    try:
        physics.reset()
        godot_x, godot_z = 100.0, -100.0
        index_x = round((godot_x / 500.0 + 0.5) * 120)
        index_z = round((godot_z / 500.0 + 0.5) * 120)
        expected = heights[index_z * 121 + index_x]
        hit = physics.ray_surface((-godot_z, -godot_x, 50.0), (0.0, 0.0, -1.0), 100.0)
        assert hit is not None
        assert 50.0 - hit.distance_m == pytest.approx(expected, abs=1e-5)
    finally:
        physics.close()
