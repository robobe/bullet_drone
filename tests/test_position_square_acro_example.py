from math import degrees, dist, hypot
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock

import pytest

from bullet_drone.controllers import AltitudeController, PidSettings, PositionController, VelocityController
from bullet_drone.controllers.internal import AttitudeController
from bullet_drone.models import DroneObservation, DroneState
from examples.position_square_acro import demo
from examples.position_square_acro.demo_cli import CliParser, DemoOptions
from examples.position_square_acro.mission import (
    Phase, SquareMission, rate_pwm, throttle_pwm,
)


DT_S = 1 / 240
MAX_THRUST_N = 34.0


def observation(time_s=0.0, position=(0.0, 0.0, 0.015), velocity=(0.0, 0.0, 0.0)):
    return DroneObservation(DroneState(
        'episode', round(time_s / DT_S), time_s, position, (0.0, 0.0, 0.0, 1.0),
        velocity, (0.0, 0.0, 0.0)))


def mission():
    return SquareMission(
        PositionController(PidSettings(.8, 0, 0, 1, 0)),
        VelocityController(1, MAX_THRUST_N, PidSettings(2, 0, 0, 1.5, 0), .261799),
        AltitudeController(1, 9.81, MAX_THRUST_N,
                           PidSettings(1.5, 0, 0, 1, 0), PidSettings(6, 4, 0, 3, 2)),
        AttitudeController((4, 4, 4), (2, 2, 2)), MAX_THRUST_N)


def test_acro_example_owns_sitl_mission_renderer_and_does_not_import_other_examples():
    directory = Path('examples/position_square_acro')
    sources = '\n'.join(path.read_text() for path in directory.glob('*.py'))
    assert 'examples.position_square.' not in sources
    assert 'examples.altitde_hold_bt.' not in sources
    for name in ('mission.py', 'sitl.py', 'renderer.py', 'diagnostics.py'):
        assert (directory / name).is_file()
    assert 'BetaflightBackend' in (directory / 'demo.py').read_text()
    assert 'SimulationControl' not in (directory / 'main.py').read_text()


def test_mission_arms_in_acro_visits_square_holds_lands_and_disarms():
    value = mission()
    value.reset(observation())
    status = SimpleNamespace(armed=False, arming_disabled_flags=0)
    assert value.command(observation(0), status, DT_S)[4:6] == (1000, 1000)
    assert value.command(observation(.5), status, DT_S)[4:6] == (2000, 1000)
    status.armed = True
    channels = value.command(observation(.6), status, DT_S)
    assert value.phase == Phase.TAKEOFF and channels[4:6] == (2000, 1000)
    now = 1.0
    for index, target in enumerate(value.waypoints):
        value.command(observation(now, target), status, DT_S)
        value.command(observation(now + 1, target), status, DT_S)
        assert value.arrivals[-1].index == index
        now += 2
    assert value.phase == Phase.HOLD
    value.command(observation(value.phase_started_s + 10, value.waypoints[-1]), status, DT_S)
    assert value.phase == Phase.LAND
    value.command(observation(now + 11, (0, 0, .015)), status, DT_S)
    channels = value.command(observation(now + 11.5, (0, 0, .015)), status, DT_S)
    assert value.phase == Phase.DISARMED and channels[2] == channels[4] == 1000
    status.armed = False
    value.command(observation(now + 12.5, (0, 0, .015)), status, DT_S)
    assert value.phase == Phase.COMPLETE
    assert [event.phase for event in value.transitions] == list(Phase)
    assert [arrival.target_position_m for arrival in value.arrivals] == [
        (0, 0, 3), (3, 0, 3), (3, 3, 3), (0, 3, 3), (0, 0, 3)]


def test_acro_rate_and_throttle_pwm_match_pinned_firmware_curves():
    profile = Path('configs/betaflight.config').read_text()
    for axis in ('roll', 'pitch', 'yaw'):
        assert f'set {axis}_rc_rate = 7' in profile
        assert f'set {axis}_srate = 67' in profile
        assert f'set {axis}_expo = 0' in profile
    assert rate_pwm(0) == 1500
    for target in (-2.0, -1.0, 1.0, 2.0):
        pwm = rate_pwm(target)
        deflection = (pwm - 1500) / 500
        actual_dps = 70 * deflection + 600 * deflection * abs(deflection)
        assert actual_dps == pytest.approx(degrees(target), abs=1.5)
    assert throttle_pwm(0, MAX_THRUST_N) == 1000
    assert throttle_pwm(MAX_THRUST_N, MAX_THRUST_N) == 2000


def test_unexpected_disarm_aborts_flight():
    value = mission()
    value.reset(observation())
    value._transition(Phase.TAKEOFF, 0)
    with pytest.raises(RuntimeError, match='disarmed during flight'):
        value.command(observation(1), SimpleNamespace(armed=False, arming_disabled_flags=0), DT_S)


def test_demo_timeout_closes_sitl_backend_and_physics(monkeypatch):
    session = MagicMock()
    session.__enter__.return_value = session
    session.__exit__.return_value = False
    session.commands.read_status.side_effect = lambda: SimpleNamespace(
        armed=False, arming_disabled_flags=1, received_at=time.monotonic())
    monkeypatch.setattr(demo, 'SitlSession', Mock(return_value=session))
    backend = Mock(received_at=None, latest=(0.0,) * 4)
    monkeypatch.setattr(demo, 'BetaflightBackend', Mock(return_value=backend))
    physics = []
    original = demo.PyBulletPhysics

    def create(*args, **kwargs):
        instance = original(*args, **kwargs)
        physics.append(instance)
        return instance

    monkeypatch.setattr(demo, 'PyBulletPhysics', create)
    with pytest.raises(TimeoutError, match='Acro square timed out'):
        demo.run_demo(max_duration_s=.01)
    session.__exit__.assert_called_once()
    backend.close.assert_called_once()
    assert physics[0]._client is None


@pytest.mark.parametrize('failure,expected', [(None, 0), (RuntimeError('failed'), 1),
                                               (KeyboardInterrupt(), 130)])
def test_cli_owns_renderer_starts_automatically_and_closes(monkeypatch, failure, expected):
    from examples.position_square_acro import main

    viewer = MagicMock()
    viewer.__enter__.return_value = viewer
    monkeypatch.setattr('examples.position_square_acro.renderer.ExternalRenderer',
                        Mock(return_value=viewer))
    initial, moving = observation(), observation(1)
    result = SimpleNamespace(observation=moving)

    def run_demo(**kwargs):
        kwargs['on_observation'](initial)
        kwargs['on_observation'](moving)
        if failure is not None:
            raise failure
        return result

    monkeypatch.setattr(demo, 'run_demo', run_demo)
    assert main.main([]) == expected
    viewer.wait_ready.assert_called_once_with(initial)
    viewer.set_state.assert_any_call(initial, 'running', 'automatic_start')
    viewer.consume.assert_any_call(moving)
    viewer.__exit__.assert_called_once()
    if failure is None:
        viewer.set_state.assert_any_call(moving, 'stopped', 'completed')


def test_missing_owned_resources_fail_before_flight(tmp_path):
    from examples.position_square_acro.renderer import ExternalRenderer
    from examples.position_square_acro.sitl import SitlSession

    with pytest.raises(FileNotFoundError, match='build.py'):
        ExternalRenderer(tmp_path / 'renderer', Path('external_render/configs/physics.json'))
    with pytest.raises(ValueError, match='setup_betaflight_sitl.sh'):
        SitlSession(tmp_path / 'sitl')


def test_cli_parser_defaults_and_help():
    assert CliParser().parse([]) == DemoOptions()
    for arguments, status in ((['--help'], 0), (['--unknown'], 2)):
        result = subprocess.run(
            [sys.executable, '-m', 'examples.position_square_acro.main', *arguments],
            capture_output=True, text=True, timeout=10)
        assert result.returncode == status, result.stderr
        assert 'pybullet build time' not in result.stderr


def test_headless_import_and_cli_do_not_load_rendering(monkeypatch):
    code = """
import builtins
original = builtins.__import__
def checked(name, *args, **kwargs):
    if name.startswith(('examples.position_square_acro.renderer', 'zmq', 'bullet_msgs')):
        raise AssertionError('unexpected renderer import: ' + name)
    return original(name, *args, **kwargs)
builtins.__import__ = checked
import examples.position_square_acro.main
"""
    result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert result.stdout == result.stderr == ''


@pytest.mark.skipif(os.environ.get('BULLET_RUN_SITL') != '1',
                    reason='Opt-in real paced Betaflight square')
def test_live_sitl_square_and_cleanup(monkeypatch):
    sessions = []
    original = demo.SitlSession

    class RecordingSession(original):
        def __enter__(self):
            value = super().__enter__()
            sessions.append(self)
            return value

    monkeypatch.setattr(demo, 'SitlSession', RecordingSession)
    samples = []
    result = demo.run_demo(on_observation=lambda value: samples.append(value.ground_truth))
    assert [event.phase for event in result.transitions] == list(Phase)
    assert [arrival.index for arrival in result.arrivals] == list(range(5))
    for arrival in result.arrivals:
        assert dist(arrival.state.position_m, arrival.target_position_m) <= .1
        assert hypot(*arrival.state.linear_velocity_m_s) <= .1
    final = result.observation.ground_truth
    assert hypot(*final.position_m[:2]) < .1 and final.position_m[2] < .02
    assert final.rotor_speeds_rad_s == (0.0,) * 4
    assert max(max(state.rotor_speeds_rad_s) for state in samples) > 0
    assert sessions[0].process.poll() is not None
    assert sessions[0].client.stop.is_set()
    assert not Path(sessions[0].work.name).exists()
