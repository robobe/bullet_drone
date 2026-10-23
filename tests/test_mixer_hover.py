from dataclasses import replace
from math import pi, sqrt

import pybullet as p
import pytest

from bullet_drone.commands import InvalidCommandError, RotorCommand, WrenchCommand
from bullet_drone.config import ConfigurationError
from bullet_drone.environment import DroneEnvironment
from bullet_drone.forces.reaction_torque import ReactionTorqueModel
from bullet_drone.forces.thrust import RotorThrustModel
from bullet_drone.mixer import MotorMixer
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.vehicle import load_racer_parameters


def test_parameters_preserve_racer_geometry_and_convert_units():
    vehicle = load_racer_parameters()
    assert vehicle.rotor_positions_m == (
        (0.085, 0.0675, 0), (-0.085, 0.0675, 0),
        (-0.085, -0.0675, 0), (0.085, -0.0675, 0),
    )
    assert vehicle.thrust_coefficient == pytest.approx(8.47e-9 * (60 / (2 * pi)) ** 2)
    assert vehicle.torque_coefficient == pytest.approx(2.13e-11 * (60 / (2 * pi)) ** 2)
    assert vehicle.max_thrust_per_rotor_n == pytest.approx(0.830 * 9.81 * 4.17 / 4)


def test_equal_thrust_and_zero_torque():
    vehicle = load_racer_parameters()
    allocation = MotorMixer(vehicle).mix(WrenchCommand(9.81))
    assert allocation.rotor_thrusts_n == pytest.approx([2.4525] * 4)
    assert allocation.speeds_rad_s == pytest.approx([1781.9332057019024] * 4)
    assert allocation.achieved.thrust_n == pytest.approx(9.81)
    assert allocation.achieved.torque_nm == pytest.approx((0, 0, 0), abs=1e-12)
    assert not allocation.saturated


@pytest.mark.parametrize("torque", [(0.01, 0, 0), (0, -0.01, 0), (0, 0, 0.001), (0.01, -0.02, 0.003)])
def test_mixer_matches_independent_force_and_torque_calculation(torque):
    vehicle = load_racer_parameters()
    allocation = MotorMixer(vehicle).mix(WrenchCommand(9.81, torque))
    forces = RotorThrustModel(vehicle.thrust_coefficient).calculate(allocation.speeds_rad_s)
    reaction = ReactionTorqueModel(vehicle.torque_coefficient, vehicle.reaction_torque_signs).calculate(allocation.speeds_rad_s)
    roll = sum(position[1] * force[2] for position, force in zip(vehicle.rotor_positions_m, forces))
    pitch = sum(-position[0] * force[2] for position, force in zip(vehicle.rotor_positions_m, forces))
    assert sum(force[2] for force in forces) == pytest.approx(9.81)
    assert (roll, pitch, reaction[2]) == pytest.approx(torque, abs=1e-12)
    assert not allocation.saturated


@pytest.mark.parametrize("desired", [
    WrenchCommand(9.81, (10, -20, 3)),
    WrenchCommand(100), WrenchCommand(100, (0.1, 0, 0)),
    WrenchCommand(0, (1, 0, 0)),
])
def test_saturation_reports_achieved_output_and_preserves_collective(desired):
    vehicle = load_racer_parameters()
    allocation = MotorMixer(vehicle).mix(desired)
    assert allocation.saturated
    assert all(0 <= thrust <= vehicle.max_thrust_per_rotor_n for thrust in allocation.rotor_thrusts_n)
    assert all(0 <= speed <= vehicle.max_speed_rad_s for speed in allocation.speeds_rad_s)
    assert allocation.achieved.thrust_n == pytest.approx(min(desired.thrust_n, 4 * vehicle.max_thrust_per_rotor_n))
    assert allocation.achieved.torque_nm == pytest.approx(
        tuple(value * allocation.torque_scale for value in desired.torque_nm), abs=1e-12,
    )
    assert 0 <= allocation.torque_scale <= 1


@pytest.mark.parametrize("kwargs", [
    {"thrust_n": -1}, {"thrust_n": True}, {"thrust_n": float("nan")},
    {"thrust_n": 1, "torque_nm": (1, 2)},
    {"thrust_n": 1, "torque_nm": (0, float("inf"), 0)},
])
def test_invalid_wrench(kwargs):
    with pytest.raises(InvalidCommandError):
        WrenchCommand(**kwargs)


@pytest.mark.parametrize("speeds", [(-1, 0, 0, 0), (0, 0, 0), (True, 0, 0, 0), (float("nan"), 0, 0, 0), (float("inf"), 0, 0, 0)])
def test_invalid_rotor_speeds(speeds):
    with pytest.raises(InvalidCommandError):
        RotorCommand(speeds, "episode", 1)


def test_uncontrollable_geometry_rejected():
    vehicle = load_racer_parameters()
    with pytest.raises(ConfigurationError, match="allocate independent"):
        MotorMixer(replace(vehicle, rotor_positions_m=((0, 0, 0),) * 4))


def test_hover_every_step_and_reset_stops_motors(config):
    vehicle = load_racer_parameters()
    allocation = MotorMixer(vehicle).mix(WrenchCommand(config.drone.mass_kg * -config.simulation.gravity_m_s2[2]))
    physics = PyBulletPhysics(config, vehicle)
    env = DroneEnvironment(physics)
    try:
        observation = env.reset()
        assert observation.ground_truth.rotor_speeds_rad_s == (0, 0, 0, 0)
        for _ in range(1200):
            state = observation.ground_truth
            observation = env.step(RotorCommand(allocation.speeds_rad_s, state.episode_id, state.step_index + 1))
            assert observation.ground_truth.position_m == pytest.approx((0, 0, 1), abs=0.001)
        state = observation.ground_truth
        assert state.simulation_time_s == 5
        assert state.orientation_xyzw == pytest.approx((0, 0, 0, 1), abs=1e-8)
        assert state.linear_velocity_m_s == pytest.approx((0, 0, 0), abs=1e-8)
        assert state.rotor_speeds_rad_s == allocation.speeds_rad_s
        reset = env.reset()
        assert reset.ground_truth.rotor_speeds_rad_s == (0, 0, 0, 0)
        assert env.step().ground_truth.position_m[2] < 1
    finally:
        env.close()


@pytest.mark.parametrize("mass", [0.5, 1.0, 2.0])
def test_mass_changes_hover_speed_but_not_motor_capacity(config, mass):
    vehicle = load_racer_parameters()
    config = replace(config, drone=replace(config.drone, mass_kg=mass))
    allocation = MotorMixer(vehicle).mix(WrenchCommand(mass * -config.simulation.gravity_m_s2[2]))
    assert allocation.speeds_rad_s == pytest.approx([1781.9332057019024 * sqrt(mass)] * 4)
    env = DroneEnvironment(PyBulletPhysics(config, vehicle))
    try:
        state = env.reset().ground_truth
        for _ in range(1200):
            state = env.step(RotorCommand(allocation.speeds_rad_s, state.episode_id, state.step_index + 1)).ground_truth
    finally:
        env.close()
    assert state.position_m[2] == pytest.approx(1, abs=0.001)
    assert vehicle.max_thrust_per_rotor_n == pytest.approx(0.830 * 9.81 * 4.17 / 4)


@pytest.mark.parametrize("torque", [(0.01, 0, 0), (0, 0.01, 0), (0, 0, -0.001)])
def test_real_roll_pitch_yaw_response_has_expected_magnitude(config, torque):
    vehicle = load_racer_parameters()
    physics = PyBulletPhysics(config, vehicle)
    allocation = MotorMixer(vehicle).mix(WrenchCommand(9.81, torque))
    try:
        initial = physics.reset()
        state = physics.step(RotorCommand(allocation.speeds_rad_s, initial.episode_id, 1))
        inertia = 0.003113 * config.drone.mass_kg / 0.830
        expected = tuple(component * config.simulation.timestep_s / inertia for component in torque)
        # Quantitative check also detects accidentally applying lever-arm torque twice.
        assert state.angular_velocity_rad_s == pytest.approx(expected, abs=1e-9)
    finally:
        physics.close()


def test_thrust_rotates_with_drone_body_frame(config):
    vehicle = load_racer_parameters()
    physics = PyBulletPhysics(config, vehicle)
    try:
        initial = physics.reset()
        p.resetBasePositionAndOrientation(
            physics._body, [0, 0, 1], p.getQuaternionFromEuler([0, pi / 2, 0]),
            physicsClientId=physics._client,
        )
        speeds = MotorMixer(vehicle).mix(WrenchCommand(9.81)).speeds_rad_s
        state = physics.step(RotorCommand(speeds, initial.episode_id, 1))
        expected_speed = 9.81 * config.simulation.timestep_s
        assert state.linear_velocity_m_s == pytest.approx((expected_speed, 0, -expected_speed), abs=1e-9)
    finally:
        physics.close()


def test_invalid_commands_never_apply_forces_or_invalidate_episode(config, monkeypatch):
    vehicle = load_racer_parameters()
    physics = PyBulletPhysics(config, vehicle)
    env = DroneEnvironment(physics)
    try:
        state = env.reset().ground_truth
        def unexpected(*args, **kwargs):
            pytest.fail("Invalid commands must not apply any forces")
        with monkeypatch.context() as patch:
            patch.setattr(p, "applyExternalForce", unexpected)
            for command in (
                RotorCommand((0, 0, 0, 0), "wrong-episode", 1),
                RotorCommand((0, 0, 0, 0), state.episode_id, 2),
                RotorCommand((vehicle.max_speed_rad_s + 1,) * 4, state.episode_id, 1),
            ):
                with pytest.raises(InvalidCommandError):
                    env.step(command)
                assert physics._state() == state
        assert env.step().ground_truth.step_index == 1
        old_episode = state.episode_id
        new_state = env.reset().ground_truth
        with pytest.raises(InvalidCommandError):
            env.step(RotorCommand((0, 0, 0, 0), old_episode, 1))
        valid = RotorCommand((0, 0, 0, 0), new_state.episode_id, 1)
        env.step(valid)
        with pytest.raises(InvalidCommandError):
            env.step(valid)
        assert env.step().ground_truth.step_index == 2
    finally:
        env.close()


def test_force_application_failure_aborts_backend(config, monkeypatch):
    vehicle = load_racer_parameters()
    physics = PyBulletPhysics(config, vehicle)
    def fail(*args, **kwargs):
        raise p.error("injected force application failure")
    monkeypatch.setattr(p, "applyExternalForce", fail)
    from bullet_drone.contracts import BackendError, LifecycleError
    try:
        physics.reset()
        with pytest.raises(BackendError, match="force application failure"):
            physics.step()
        with pytest.raises(LifecycleError):
            physics.step()
    finally:
        physics.close()
    assert physics._client is None
    with pytest.raises(LifecycleError):
        physics.step()
