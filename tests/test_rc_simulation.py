from dataclasses import FrozenInstanceError
from unittest.mock import Mock

import pytest

from bullet_drone.commands import HighLevelCommand, InvalidCommandError, RcInput, RotorCommand, WrenchCommand
from bullet_drone.config import DroneSettings, SimulationConfig, SimulationSettings
from bullet_drone.contracts import LifecycleError
from bullet_drone.controllers import InternalControlSettings, PidSettings
from bullet_drone.simulation import RcSimulation


def simulation(**kwargs):
    config = SimulationConfig(SimulationSettings(1/240, 20, (0, 0, -9.81)),
                              DroneSettings(1, (0, 0, 1)))
    settings = InternalControlSettings((4, 4, 4), (2, 2, 2), (PidSettings(.05, 0, 0, .1, 0),)*3)
    return RcSimulation(config, settings, **kwargs)


def test_one_rc_step_feedback_identity_disarm_and_reset():
    sim = simulation()
    try:
        with pytest.raises(LifecycleError): sim.step(RcInput())
        before = sim.reset()
        assert sim.last_command is None and sim.allocation is None
        after = sim.step(sim._rc.codec.encode('angle', (0, 0, 0), 9.81, True))
        assert after.ground_truth.step_index == before.ground_truth.step_index + 1 == 1
        assert after.ground_truth.episode_id == before.ground_truth.episode_id
        assert after.ground_truth.simulation_time_s == sim.timestep_s
        assert sim.last_command == sim._rc.codec.decode(sim._rc.codec.encode('angle', (0, 0, 0), 9.81, True), before.ground_truth.episode_id, 1)
        assert sim.allocation is not None
        with pytest.raises(FrozenInstanceError): after.ground_truth.position_m = (1, 2, 3)
        disarmed = sim.step(sim._rc.codec.encode('angle', (1, 1, 1), 20, False))
        assert disarmed.ground_truth.rotor_speeds_rad_s == (0, 0, 0, 0)
        assert sim.allocation is None and not sim.last_command.armed
        reset = sim.reset()
        assert reset.ground_truth.episode_id != before.ground_truth.episode_id
        assert reset.ground_truth.step_index == 0 and sim.last_command is None
        assert before.ground_truth.step_index == 0
    finally:
        sim.close()
    sim.close()
    assert sim._environment._physics._client is None and sim._rc._closed
    with pytest.raises(LifecycleError): sim.reset()
    with pytest.raises(LifecycleError): sim.step(RcInput())


@pytest.mark.parametrize('make_command', [
    lambda episode: None,
    lambda episode: RotorCommand((0, 0, 0, 0), episode, 1),
    lambda episode: WrenchCommand(10),
    lambda episode: HighLevelCommand('angle', (0, 0, 0), 10, episode, 1),
])
def test_rejects_non_rc_without_consuming_a_step(make_command):
    sim = simulation()
    try:
        observation = sim.reset()
        spy = Mock(wraps=sim._environment.step)
        sim._environment.step = spy
        with pytest.raises(InvalidCommandError):
            sim.step(make_command(observation.ground_truth.episode_id))
        spy.assert_not_called()
        assert sim.last_command is None
        assert sim.step(RcInput()).ground_truth.step_index == 1
        spy.assert_called_once()
    finally:
        sim.close()


def test_revalidates_corrupted_input_without_consuming_step():
    sim = simulation()
    try:
        sim.reset()
        command = RcInput()
        object.__setattr__(command, 'channels', (999,)*18)
        with pytest.raises(InvalidCommandError): sim.step(command)
        assert sim.step(RcInput()).ground_truth.step_index == 1
    finally:
        sim.close()


@pytest.mark.parametrize('stage', ['control', 'physics'])
@pytest.mark.parametrize('failure', [RuntimeError('injected'), KeyboardInterrupt()])
def test_failure_requires_reset_and_cleanup(stage, failure, monkeypatch):
    sim = simulation()
    try:
        sim.reset()
        target = sim._provider if stage == 'control' else sim._environment
        method = 'command' if stage == 'control' else 'step'
        original = getattr(target, method)
        monkeypatch.setattr(target, method, Mock(side_effect=failure))
        with pytest.raises(type(failure)): sim.step(RcInput())
        assert sim.last_command is None and sim.allocation is None
        with pytest.raises(LifecycleError): sim.step(RcInput())
        monkeypatch.setattr(target, method, original)
        sim.reset()
        assert sim.step(RcInput()).ground_truth.step_index == 1
    finally:
        sim.close()
    assert sim._environment._physics._client is None


def test_close_attempts_both_resources_even_when_one_fails():
    sim = simulation()
    sim._provider.close = Mock(side_effect=RuntimeError('provider cleanup'))
    sim._environment.close = Mock(side_effect=ValueError('physics cleanup'))
    with pytest.raises(RuntimeError, match='provider cleanup') as error:
        sim.close()
    sim._environment.close.assert_called_once()
    assert 'physics cleanup' in error.value.__notes__[0]
    sim.close()


@pytest.mark.parametrize('mode', ['angle', 'acro'])
def test_real_rc_yaw_turn_rate_and_neutral_stop(mode):
    from bullet_drone.controllers import IdealStateFeedback
    from bullet_drone.geometry import yaw_from_quaternion
    sim = simulation()
    try:
        sim.reset()
        turn = sim._rc.codec.encode(mode, (0, 0, .4), 9.81, True)
        for _ in range(240):
            observation = sim.step(turn)
        feedback = IdealStateFeedback().read(observation)
        assert feedback.body_rates_rad_s[2] == pytest.approx(.4, abs=.01)
        assert yaw_from_quaternion(observation.ground_truth.orientation_xyzw) > .25
        neutral = sim._rc.codec.encode(mode, (0, 0, 0), 9.81, True)
        for _ in range(240):
            observation = sim.step(neutral)
        assert abs(IdealStateFeedback().read(observation).body_rates_rad_s[2]) < .01
        assert yaw_from_quaternion(observation.ground_truth.orientation_xyzw) > .25
    finally:
        sim.close()
