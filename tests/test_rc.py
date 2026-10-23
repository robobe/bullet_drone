from dataclasses import replace
from unittest.mock import Mock

import pytest

from bullet_drone.commands import HighLevelCommand, InvalidCommandError, RcInput
from bullet_drone.composition import FlightCommandProvider
from bullet_drone.contracts import LifecycleError
from bullet_drone.rc import RcCodec, SoftwareRc
from bullet_drone.models import DroneObservation, DroneState


def observation(step=0, episode='episode'):
    return DroneObservation(DroneState(episode, step, step * 0.1, (0, 0, 0),
                                      (0, 0, 0, 1), (0, 0, 0), (0, 0, 0)))


@pytest.mark.parametrize('mode', ['angle', 'acro'])
def test_rc_translation_reset_and_lifecycle(mode):
    rc = SoftwareRc(RcCodec(30))
    with pytest.raises(LifecycleError):
        rc.set_input(rc.codec.encode())
    rc.reset(observation())
    default = rc.command(observation(), 0.1)
    assert not default.armed and default.thrust_n == 0
    rc.set_input(rc.codec.encode(mode, (0.1, -0.2, 0.3), 12, True))
    command = rc.command(observation(1), 0.1)
    assert command == rc.codec.decode(rc.codec.encode(mode, (.1, -.2, .3), 12, True), 'episode', 2)
    with pytest.raises(InvalidCommandError):
        rc.command(observation(1), 0.1)
    rc.reset(observation(0, 'new'))
    assert not rc.command(observation(0, 'new'), 0.1).armed
    rc.close()
    rc.close()
    with pytest.raises(LifecycleError):
        rc.reset(observation())
    with pytest.raises(LifecycleError):
        rc.command(observation(1), 0.1)


@pytest.mark.parametrize('channels', [(), (1500,)*17, (1500,)*19,
    (True,) + (1500,)*17, (1500.,) + (1500,)*17,
    (999,) + (1500,)*17, (2001,) + (1500,)*17,
    (float('nan'),) + (1500,)*17, None])
def test_rc_rejects_invalid_input(channels):
    with pytest.raises(InvalidCommandError):
        RcInput(channels)


@pytest.mark.parametrize('kwargs', [dict(episode='wrong'), dict(step=1)])
def test_rc_rejects_identity_and_invalid_dt_without_consuming_step(kwargs):
    rc = SoftwareRc(RcCodec(30))
    rc.reset(observation())
    with pytest.raises(InvalidCommandError):
        rc.command(observation(**kwargs), 0.1)
    with pytest.raises(InvalidCommandError):
        rc.command(observation(), 0)
    with pytest.raises(InvalidCommandError):
        rc.set_input(None)
    assert rc.command(observation(), 0.1).step_index == 1


def test_disarm_gates_nonzero_demands_and_rearm_resets_stabilization():
    rc = SoftwareRc(RcCodec(30))
    stabilization, feedback, mixer = Mock(), Mock(), Mock()
    mixer.mix.return_value.speeds_rad_s = (10, 20, 30, 40)
    provider = FlightCommandProvider(rc, stabilization, feedback, mixer, 0.1)
    provider.reset(observation())
    rc.set_input(rc.codec.encode('angle', (1, 2, 3), 30, False))
    assert provider.command(observation()).speeds_rad_s == (0, 0, 0, 0)
    stabilization.update.assert_not_called()
    feedback.read.assert_not_called()
    mixer.mix.assert_not_called()
    assert provider.allocation is None
    resets = stabilization.reset.call_count
    rc.set_input(rc.codec.encode('angle', (0, 0, 0), 10, True))
    assert provider.command(observation(1)).speeds_rad_s == (10, 20, 30, 40)
    assert stabilization.reset.call_count == resets + 1
    rc.set_input(rc.codec.encode('angle', (1, 2, 3), 30, False))
    assert provider.command(observation(2)).speeds_rad_s == (0, 0, 0, 0)
    assert stabilization.reset.call_count == resets + 2
    assert stabilization.update.call_count == 1
    rc.set_input(rc.codec.encode('angle', (0, 0, 0), 10, True))
    provider.command(observation(3))
    assert stabilization.reset.call_count == resets + 3
    provider.close()


def test_disarmed_command_is_validated_before_motor_gate():
    source, stabilization = Mock(), Mock()
    source.command.return_value = HighLevelCommand('angle', (0, 0, 0), 10, 'wrong', 1, False)
    provider = FlightCommandProvider(source, stabilization, Mock(), Mock(), 0.1)
    provider.reset(observation())
    resets = stabilization.reset.call_count
    with pytest.raises(InvalidCommandError):
        provider.command(observation())
    assert stabilization.reset.call_count == resets
    source.command.return_value = replace(source.command.return_value, episode_id='episode')
    assert provider.command(observation()).speeds_rad_s == (0, 0, 0, 0)


@pytest.mark.parametrize('mode', ['angle', 'acro'])
def test_aetr_endpoints_center_quantization_and_aux(mode):
    from math import pi
    codec = RcCodec(40, (3, 4, 5))
    frame = codec.encode(mode, (100, -100, 100), 100, True)
    assert frame.channels == (2000, 1000, 2000, 2000, 2000,
                              2000 if mode == 'angle' else 1000) + (1000,) * 12
    command = codec.decode(frame, 'e', 1)
    assert command.armed and command.mode == mode and command.thrust_n == 40
    assert command.rpy[:2] == pytest.approx((pi/4, -pi/4) if mode == 'angle' else (3, -4))
    assert (command.yaw_rate_rad_s if mode == 'angle' else command.rpy[2]) == 5
    neutral = codec.decode(codec.encode(mode), 'e', 1)
    assert neutral.rpy == (0, 0, 0) and neutral.thrust_n == 0 and not neutral.armed
    frame = codec.encode(mode, (.123, -.234, .345), 9.81, True)
    command = codec.decode(frame, 'e', 1)
    limits = (pi/4, pi/4, 5) if mode == 'angle' else (3, 4, 5)
    actual = (*command.rpy[:2], command.yaw_rate_rad_s if mode == 'angle' else command.rpy[2])
    for value, desired, limit in zip(actual, (.123, -.234, .345), limits):
        assert abs(value-desired) <= limit/1000 + 1e-12
    assert abs(command.thrust_n-9.81) <= .02
    assert all(type(v) is int for v in frame.channels)


def test_reassigned_channels_thresholds_and_unused_channels():
    from bullet_drone.config import RcSettings
    settings = RcSettings(aetr_channels=(4, 3, 2, 1), arm_channel=18, mode_channel=17,
                          arm_threshold=1800, angle_threshold=1600, max_angle_rad=.3)
    codec = RcCodec(30, (1, 2, 3), settings)
    frame = codec.encode('angle', (.3, -.3, 3), 30, True)
    assert frame.channels[:4] == (2000, 2000, 1000, 2000)
    assert frame.channels[4:16] == (1000,)*12
    channels = list(frame.channels)
    channels[17], channels[16] = 1799, 1599
    command = codec.decode(RcInput(channels), 'e', 1)
    assert not command.armed and command.mode == 'acro'
    channels[17], channels[16] = 1800, 1600
    command = codec.decode(RcInput(channels), 'e', 1)
    assert command.armed and command.mode == 'angle'
    channels[8] = 2000
    assert codec.decode(RcInput(channels), 'e', 1) == command
    assert not codec.decode(RcInput(), 'e', 1).armed


@pytest.mark.parametrize('kwargs', [dict(aetr_channels=(1, 2, 3)), dict(aetr_channels=(1, 1, 3, 4)),
    dict(arm_channel=4), dict(mode_channel=5), dict(arm_channel=True), dict(mode_channel=19),
    dict(arm_threshold=1000), dict(angle_threshold=2001), dict(max_angle_rad=0),
    dict(max_angle_rad=float('nan')), dict(max_angle_rad=2)])
def test_invalid_rc_configuration(kwargs):
    from bullet_drone.config import ConfigurationError, RcSettings
    with pytest.raises(ConfigurationError): RcSettings(**kwargs)


def test_yaml_rc_configuration_and_unknown_keys(tmp_path):
    from bullet_drone.config import ConfigurationError, YamlConfigLoader
    path = tmp_path/'rc.yaml'
    base = 'simulation: {timestep_s: 0.01, max_steps: 2, gravity_m_s2: [0, 0, -9.81]}\ndrone: {mass_kg: 1, initial_position_m: [0, 0, 1]}\n'
    path.write_text(base + 'rc: {aetr_channels: [4, 3, 2, 1], arm_channel: 7, mode_channel: 8, arm_threshold: 1700}\n')
    settings = YamlConfigLoader().load(path).rc
    assert settings.aetr_channels == (4, 3, 2, 1) and settings.arm_channel == 7 and settings.arm_threshold == 1700
    path.write_text(base + 'rc: {unknown: 1}\n')
    with pytest.raises(ConfigurationError): YamlConfigLoader().load(path)
    path.write_text(base)
    assert YamlConfigLoader().load(path).rc.arm_channel == 5


def test_heading_control_wraps_and_limits():
    from math import pi
    from bullet_drone.rc import heading_rate
    assert heading_rate(-pi+.1, pi-.1, 2) == pytest.approx(.8)
    assert heading_rate(pi-.1, -pi+.1, 2) == pytest.approx(-.8)
    assert heading_rate(pi/2, 0, 2) == 2
    assert heading_rate(1, 1, 2) == 0


def test_angle_yaw_rate_bypasses_heading_and_mode_changes_reset_history():
    from math import pi
    from bullet_drone.controllers import InternalControlSettings, InternalFlightController, PidSettings
    from bullet_drone.geometry import from_rpy
    from bullet_drone.models import ControlFeedback
    settings = InternalControlSettings((4,4,4), (2,2,2), (PidSettings(.05,.01,0,.1,.05),)*3)
    control = InternalFlightController(settings)
    codec = RcCodec(30)
    control.rate.torque = Mock(wraps=control.rate.torque)
    feedback = ControlFeedback(from_rpy((0,0,pi/2)), (0,0,0))
    command = codec.decode(codec.encode('angle', (.1,-.2,.5), 10, True), 'e', 1)
    control.update(command, feedback, .1)
    rates = control.rate.torque.call_args.args[0]
    assert rates[0] > 0 and rates[1] < 0 and rates[2] == .5
    # Neutral yaw is zero rate even at a nonzero world heading.
    command = codec.decode(codec.encode('angle', (0,0,0), 10, True), 'e', 2)
    control.update(command, feedback, .1)
    assert control.rate.torque.call_args.args[0] == pytest.approx((0,0,0))
    control.rate.reset = Mock(wraps=control.rate.reset)
    command = codec.decode(codec.encode('acro', (.3,-.4,.5), 10, True), 'e', 3)
    control.update(command, feedback, .1)
    control.rate.reset.assert_called_once()
    assert control.rate.torque.call_args.args[0] == pytest.approx((.3,-.4,.5))
    # Legacy internal SI angle commands still hold an absolute heading.
    control.update(HighLevelCommand('angle', (0,0,0), 10, 'e', 4), feedback, .1)
    assert control.rate.torque.call_args.args[0][2] == -2


@pytest.mark.parametrize('value', [float('nan'), True])
def test_invalid_internal_yaw_rate(value):
    with pytest.raises(InvalidCommandError):
        HighLevelCommand('angle', (0,0,0), 10, 'e', 1, True, value)
    with pytest.raises(InvalidCommandError):
        HighLevelCommand('acro', (0,0,0), 10, 'e', 1, True, 0)
