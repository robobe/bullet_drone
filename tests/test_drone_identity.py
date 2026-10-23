from dataclasses import FrozenInstanceError, replace

import pytest

from bullet_drone.environment import DroneEnvironment
from bullet_drone.physics import PyBulletPhysics


def test_config_identity_survives_step_and_episode_reset(config):
    config = replace(config, drone=replace(config.drone, drone_id='drone-alpha'))
    env = DroneEnvironment(PyBulletPhysics(config))
    try:
        initial = env.reset().ground_truth
        stepped = env.step().ground_truth
        reset = env.reset().ground_truth
        assert initial.drone_id == stepped.drone_id == reset.drone_id == 'drone-alpha'
        assert stepped.episode_id == initial.episode_id != reset.episode_id
        assert reset.step_index == 0
        with pytest.raises(FrozenInstanceError):
            initial.drone_id = 'renamed'
    finally:
        env.close()
