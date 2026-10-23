from dataclasses import replace
from math import pi
from unittest.mock import Mock

import pybullet as p
import pytest
import yaml

from bullet_drone.commands import HighLevelCommand, InvalidCommandError
from bullet_drone.composition import UnsupportedControllerError, create_flight_command_provider
from bullet_drone.config import ConfigurationError, ControlSettings, YamlConfigLoader
from bullet_drone.contracts import LifecycleError
from bullet_drone.controllers import InternalControlSettings, InternalFlightController, IdealStateFeedback, PidSettings
from bullet_drone.geometry import attitude_error, from_rpy
from bullet_drone.environment import DroneEnvironment
from bullet_drone.models import ControlFeedback, DroneObservation, DroneState
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.vehicle import load_racer_parameters


def settings(**overrides):
    values = dict(attitude_kp=(4, 4, 4), max_rates_rad_s=(2, 2, 2),
                  rate_pid=(PidSettings(0.05, 0, 0, 0.1, 0),) * 3)
    values.update(overrides)
    return InternalControlSettings(**values)


def observation(step=0):
    return DroneObservation(DroneState('episode', step, step / 240, (0, 0, 1), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0)))


class Source:
    """Test-only high-level source; future hover controller supplies this contract."""
    def __init__(self, mode='angle', rpy=(0, 0, 0)):
        self.mode, self.rpy = mode, rpy
        self.resets = 0
        self.closed = False

    def reset(self, observation):
        self.resets += 1

    def command(self, observation, dt_s):
        s = observation.ground_truth
        return HighLevelCommand(self.mode, self.rpy, 9.81, s.episode_id, s.step_index + 1)

    def close(self):
        self.closed = True


def provider(source, config):
    return create_flight_command_provider(config.control, source, settings(), load_racer_parameters(), config.simulation.timestep_s)


def test_selector_defaults_validation_and_unsupported_before_resources(tmp_path):
    assert YamlConfigLoader().load('configs/basic.yaml').control.backend == 'internal'
    data = yaml.safe_load(open('configs/basic.yaml'))
    data['control'] = {'backend': 'betaflight'}
    path = tmp_path / 'bf.yaml'
    path.write_text(yaml.safe_dump(data))
    assert YamlConfigLoader().load(path).control.backend == 'betaflight'
    source = Mock()
    with pytest.raises(UnsupportedControllerError):
        create_flight_command_provider(ControlSettings('betaflight'), source, settings(), load_racer_parameters(), 1/240)
    source.reset.assert_not_called()
    for value in ('unknown', True, []):
        with pytest.raises(ConfigurationError):
            ControlSettings(value)
    data['control'] = {'backend': 'internal', 'extra': 1}
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ConfigurationError):
        YamlConfigLoader().load(path)


@pytest.mark.parametrize('kwargs', [dict(mode='bad'), dict(rpy=(0, 0)), dict(rpy=(0, True, 0)),
                                    dict(thrust_n=-1), dict(thrust_n=float('nan')),
                                    dict(episode_id=''), dict(step_index=0)])
def test_invalid_high_level_commands(kwargs):
    values = dict(mode='angle', rpy=(0, 0, 0), thrust_n=9.81, episode_id='episode', step_index=1)
    values.update(kwargs)
    with pytest.raises(InvalidCommandError):
        HighLevelCommand(**values)


@pytest.mark.parametrize('kwargs', [dict(attitude_kp=(-1, 0, 0)), dict(max_rates_rad_s=(0, 1, 1)),
                                    dict(rate_feedforward=(0, float('inf'), 0)), dict(rate_pid=()),
                                    dict(rate_pid=(1, 1, 1))])
def test_invalid_controller_settings(kwargs):
    with pytest.raises(ConfigurationError):
        settings(**kwargs)


def test_ideal_feedback_rotates_world_rates_to_body_without_mutation():
    obs = observation()
    state = replace(obs.ground_truth, orientation_xyzw=from_rpy((0, 0, pi/2)), angular_velocity_rad_s=(0, 2, 0))
    obs = DroneObservation(state)
    feedback = IdealStateFeedback().read(obs)
    assert feedback.body_rates_rad_s == pytest.approx((2, 0, 0), abs=1e-12)
    assert obs.ground_truth == state
    assert obs.sensors.imu is None


def test_angle_acro_thrust_bypass_limits_and_feedforward(monkeypatch):
    controller = InternalFlightController(settings(rate_feedforward=(0.01, 0, 0)))
    feedback = ControlFeedback((0, 0, 0, 1), (0, 0, 0))
    command = HighLevelCommand('angle', (0.1, 0, 0), 8, 'episode', 1)
    wrench = controller.update(command, feedback, 1/240)
    assert wrench.thrust_n == 8
    assert wrench.torque_nm == pytest.approx((0.024, 0, 0))
    assert controller.update(replace(command, thrust_n=12), feedback, 1/240).torque_nm == pytest.approx(wrench.torque_nm)
    def unexpected(*args):
        pytest.fail('Acro must bypass attitude controller')
    monkeypatch.setattr(controller.attitude, 'rates', unexpected)
    wrench = controller.update(replace(command, mode='acro', rpy=(10, -10, 0)), feedback, 1/240)
    assert wrench.torque_nm == pytest.approx((0.1, -0.1, 0))


def test_provider_identity_replay_reset_and_invalid_command(config):
    source = Source()
    control = provider(source, config)
    with pytest.raises(LifecycleError):
        control.command(observation())
    control.reset(observation())
    source.command = Mock(return_value=HighLevelCommand('acro', (0, 0, 0), 9.81, 'wrong', 1))
    original = control._controller.update
    control._controller.update = Mock(wraps=original)
    with pytest.raises(InvalidCommandError):
        control.command(observation())
    control._controller.update.assert_not_called()
    source.command.return_value = HighLevelCommand('acro', (0, 0, 0), 9.81, 'episode', 1)
    output = control.command(observation())
    assert output.episode_id == 'episode' and output.step_index == 1
    with pytest.raises(InvalidCommandError):
        control.command(observation())
    control.reset(observation())
    assert control.command(observation()) == output
    control.close()
    control.close()
    assert source.closed
    with pytest.raises(LifecycleError):
        control.reset(observation())


def test_pid_history_cleared_on_episode_and_mode_change(config):
    source = Source('acro', (0.1, 0, 0))
    cfg = settings(rate_pid=(PidSettings(0, 0.1, 0, 0.1, 0.1),)*3)
    control = create_flight_command_provider(config.control, source, cfg, load_racer_parameters(), 0.1)
    control.reset(observation())
    control.command(observation())
    assert control._controller.rate.pids[0].integral == pytest.approx(0.001)
    control.reset(observation())
    assert control._controller.rate.pids[0].integral == 0
    c = InternalFlightController(cfg)
    feedback = ControlFeedback((0, 0, 0, 1), (0, 0, 0))
    c.update(HighLevelCommand('acro', (0.1, 0, 0), 9.81, 'episode', 1), feedback, 0.1)
    c.update(HighLevelCommand('angle', (0, 0, 0), 9.81, 'episode', 2), feedback, 0.1)
    assert c.rate.pids[0].integral == 0


@pytest.mark.parametrize('mode', ['angle', 'acro'])
@pytest.mark.parametrize('axis', [0, 1, 2])
def test_internal_pipeline_stabilizes_small_disturbances(config, mode, axis):
    vehicle = load_racer_parameters()
    physics = PyBulletPhysics(config, vehicle)
    env = DroneEnvironment(physics)
    source = Source(mode)
    control = provider(source, config)
    try:
        obs = env.reset()
        initial = [0, 0, 0]
        initial[axis] = 0.08 if mode == 'angle' else 0.2
        if mode == 'angle':
            p.resetBasePositionAndOrientation(physics._body, [0, 0, 1], from_rpy(initial), physicsClientId=physics._client)
        else:
            p.resetBaseVelocity(physics._body, angularVelocity=initial, physicsClientId=physics._client)
        obs = DroneObservation(physics._state())
        control.reset(obs)
        for _ in range(240):
            obs = env.step(control.command(obs))
        feedback = IdealStateFeedback().read(obs)
        if mode == 'angle':
            assert abs(attitude_error(feedback.orientation_xyzw, (0, 0, 0, 1))[axis]) < 0.002
        assert abs(feedback.body_rates_rad_s[axis]) < 0.01
        assert obs.ground_truth.step_index == 240
        assert control.allocation.requested.thrust_n == 9.81
    finally:
        control.close()
        env.close()


def test_provider_runtime_failure_requires_reset(config):
    source = Source()
    control = provider(source, config)
    control.reset(observation())
    control._feedback = Mock()
    control._feedback.read.side_effect = RuntimeError('feedback failed')
    with pytest.raises(RuntimeError):
        control.command(observation())
    with pytest.raises(LifecycleError):
        control.command(observation())


def test_invalid_feedback_and_dt_do_not_change_pid_history():
    controller = InternalFlightController(settings())
    command = HighLevelCommand('acro', (0.1, 0, 0), 9.81, 'episode', 1)
    feedback = ControlFeedback((0, 0, 0, 1), (0, 0, 0))
    controller.update(command, feedback, 0.1)
    old = [(pid.integral, pid._previous_measurement) for pid in controller.rate.pids]
    for bad, dt in [(replace(feedback, orientation_xyzw=(0,)*4), 0.1),
                    (replace(feedback, body_rates_rad_s=(0, 0, float('nan'))), 0.1), (feedback, 0)]:
        with pytest.raises(InvalidCommandError):
            controller.update(command, bad, dt)
        assert [(pid.integral, pid._previous_measurement) for pid in controller.rate.pids] == old
