from dataclasses import FrozenInstanceError, replace

import pybullet as p
import pytest

from bullet_drone.contracts import BackendError, LifecycleError
from bullet_drone.environment import DroneEnvironment
from bullet_drone.physics import PyBulletPhysics


def test_free_fall_reset_immutable_observation_and_lifecycle(config):
    physics = PyBulletPhysics(config)
    env = DroneEnvironment(physics)
    with pytest.raises(LifecycleError):
        env.step()
    try:
        initial = env.reset()
        assert initial.ground_truth.position_m == (0, 0, 1)
        assert initial.ground_truth.orientation_xyzw == (0, 0, 0, 1)
        assert initial.ground_truth.linear_velocity_m_s == (0, 0, 0)
        assert initial.sensors.imu is initial.sensors.barometer is initial.sensors.rangefinder is None
        with pytest.raises(FrozenInstanceError):
            initial.ground_truth.step_index = 4
        with pytest.raises(ValueError):
            env.step([1, 2, 3, 4])
        first = env.step().ground_truth
        assert first.step_index == 1
        assert first.simulation_time_s == config.simulation.timestep_s
        for _ in range(23):
            state = env.step().ground_truth
        # Semi-implicit Euler error over 0.1 s is about 2 mm at 240 Hz.
        assert state.position_m[2] == pytest.approx(1 - 0.5 * 9.81 * 0.1 ** 2, abs=0.003)
        assert state.linear_velocity_m_s[2] == pytest.approx(-0.981, abs=1e-6)
        assert initial.ground_truth.step_index == 0
        assert initial.ground_truth.position_m == (0, 0, 1)
        reset = env.reset().ground_truth
        assert reset.episode_id != initial.ground_truth.episode_id
        assert reset.step_index == 0 and reset.simulation_time_s == 0
        assert reset.position_m == (0, 0, 1)
        assert reset.linear_velocity_m_s == reset.angular_velocity_rad_s == (0, 0, 0)
    finally:
        env.close()
    env.close()
    with pytest.raises(LifecycleError):
        env.reset()
    with pytest.raises(LifecycleError):
        env.step()


def test_ground_contact(config):
    physics = PyBulletPhysics(config)
    env = DroneEnvironment(physics)
    try:
        env.reset()
        for _ in range(config.simulation.max_steps):
            result = env.step()
    finally:
        env.close()
    state = result.ground_truth
    assert state.step_index == 1200
    assert state.simulation_time_s == 5
    # Racer cylinder center settles at its 12.5 mm half-height, within 1 mm.
    assert state.position_m[2] == pytest.approx(0.0125, abs=0.001)
    assert abs(state.linear_velocity_m_s[2]) < 0.001
    with pytest.raises(LifecycleError):
        physics.step()


@pytest.mark.parametrize("failure", [p.error("injected shape failure"), KeyboardInterrupt()])
def test_initialization_failure_disconnects_owned_client(config, monkeypatch, failure):
    clients = []
    original_connect = p.connect
    def connect(*args, **kwargs):
        client = original_connect(*args, **kwargs)
        clients.append(client)
        return client
    monkeypatch.setattr(p, "connect", connect)
    def fail(*args, **kwargs):
        raise failure
    monkeypatch.setattr(p, "createCollisionShape", fail)
    physics = PyBulletPhysics(config)
    with pytest.raises(KeyboardInterrupt if isinstance(failure, KeyboardInterrupt) else BackendError):
        physics.reset()
    assert len(clients) == 1 and not p.isConnected(clients[0])
    physics.close()


@pytest.mark.parametrize("failure", [p.error("injected step failure"), KeyboardInterrupt()])
def test_failed_step_requires_reset(config, monkeypatch, failure):
    physics = PyBulletPhysics(config)
    initial = physics.reset()
    original_step = p.stepSimulation
    def fail(*args, **kwargs):
        raise failure
    monkeypatch.setattr(p, "stepSimulation", fail)
    try:
        with pytest.raises(KeyboardInterrupt if isinstance(failure, KeyboardInterrupt) else BackendError):
            physics.step()
        with pytest.raises(LifecycleError):
            physics.step()
        monkeypatch.setattr(p, "stepSimulation", original_step)
        reset = physics.reset()
        assert reset.episode_id != initial.episode_id
        assert physics.step().step_index == 1
    finally:
        physics.close()


def test_racer_model_colors_mass_and_inertia(config):
    physics = PyBulletPhysics(config)
    try:
        physics.reset()
        client, body = physics._client, physics._body
        mass, _, inertia, *_ = p.getDynamicsInfo(body, -1, physicsClientId=client)
        assert mass == config.drone.mass_kg
        assert inertia == pytest.approx([0.003113 * mass / 0.830] * 3)
        links = {
            p.getJointInfo(body, i, physicsClientId=client)[12].decode(): i
            for i in range(p.getNumJoints(body, physicsClientId=client))
        }
        visuals = {shape[1]: shape for shape in p.getVisualShapeData(body, physicsClientId=client)}
        for name in ("prop0_link", "prop3_link"):
            assert visuals[links[name]][7] == pytest.approx((1, 0.18, 0.02, 1))
        for name in ("prop1_link", "prop2_link"):
            assert visuals[links[name]][7] == pytest.approx((0.05, 0.3, 0.9, 1))
        assert all(shape[2] == p.GEOM_MESH for shape in visuals.values())
        assert len(visuals) == 5
    finally:
        physics.close()


def test_urdf_load_failure_disconnects(config, monkeypatch):
    clients = []
    original_connect = p.connect
    def connect(*args, **kwargs):
        client = original_connect(*args, **kwargs)
        clients.append(client)
        return client
    def fail(*args, **kwargs):
        raise p.error("injected URDF load failure")
    monkeypatch.setattr(p, "connect", connect)
    monkeypatch.setattr(p, "loadURDF", fail)
    physics = PyBulletPhysics(config)
    with pytest.raises(BackendError, match="URDF load failure"):
        physics.reset()
    assert len(clients) == 1 and not p.isConnected(clients[0])
    physics.close()


def test_lost_connection_ends_episode(config):
    physics = PyBulletPhysics(config)
    env = DroneEnvironment(physics)
    env.reset()
    p.disconnect(physicsClientId=physics._client)
    with pytest.raises(BackendError, match="connection was lost"):
        env.step()
    with pytest.raises(LifecycleError):
        env.step()
    env.close()


def test_gui_without_display_fails_before_native_connect(config, monkeypatch):
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.setattr("bullet_drone.physics.sys.platform", "linux")
    def unexpected(*args, **kwargs):
        pytest.fail("Native GUI connect must not be called without a display")
    monkeypatch.setattr(p, "connect", unexpected)
    config = replace(config, simulation=replace(config.simulation, gui=True))
    physics = PyBulletPhysics(config)
    with pytest.raises(BackendError, match="DISPLAY"):
        physics.reset()
    physics.close()


def test_gui_connection_selection_and_setup(config, monkeypatch):
    # Use a real DIRECT world behind the requested GUI connection in CI.
    original_connect = p.connect
    requested = []
    monkeypatch.setenv("DISPLAY", ":test")
    def connect(mode):
        requested.append(mode)
        return original_connect(p.DIRECT)
    monkeypatch.setattr(p, "connect", connect)
    config = replace(config, simulation=replace(config.simulation, gui=True))
    physics = PyBulletPhysics(config)
    try:
        physics.reset()
        assert requested == [p.GUI]
        assert physics.step().step_index == 1
    finally:
        physics.close()
