from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock

import pytest

from bullet_drone.commands import InvalidCommandError
from bullet_drone.controllers import AltitudeController, PidSettings, PositionController, VelocityController
from bullet_drone.controllers.internal import AttitudeController
from bullet_drone.geometry import from_rpy
from bullet_drone.models import DroneObservation, DroneState, RangefinderReading, SensorReadings
from examples.terrain_following_bt import demo
from examples.terrain_following_bt.cli import CliParser, DemoOptions
from examples.terrain_following_bt.mission import (
    Phase, RETURN_LANE_OFFSET_M, TARGET_CLEARANCE_M, TerrainMission, TURN_RADIUS_M,
)


DT_S = 0.1
MAX_THRUST_N = 34.0


def observation(time_s=0.0, position=(0.0, 0.0, 0.015), velocity=(0.0, 0.0, 0.0),
                clearance_m=0.015, yaw_rad=0.0, *, valid=True, episode='episode'):
    step = round(time_s / DT_S)
    state = DroneState(episode, step, time_s, position, from_rpy((0.0, 0.0, yaw_rad)),
                       velocity, (0.0, 0.0, 0.0))
    reading = RangefinderReading(episode, step, time_s,
                                 clearance_m * 100.0 if valid else None, valid)
    return DroneObservation(state, SensorReadings(rangefinder=reading))


def mission():
    return TerrainMission(
        PositionController(PidSettings(.8, 0, 0, 1.5, 0)),
        VelocityController(1, MAX_THRUST_N, PidSettings(2, 0, 0, 1.5, 0), .261799),
        AltitudeController(1, 9.81, MAX_THRUST_N,
                           PidSettings(1.5, 0, 0, 1.5, 0), PidSettings(6, 4, 0, 3, 2)),
        AttitudeController((4, 4, 4), (2, 2, 2)), MAX_THRUST_N)


def test_example_is_self_contained_and_selectable():
    directory = Path('examples/terrain_following_bt')
    sources = '\n'.join(path.read_text() for path in directory.glob('*.py'))
    assert 'examples.position_square_acro.' not in sources
    assert 'examples.altitde_hold_bt.' not in sources
    task = Path('.vscode/tasks.json').read_text()
    assert 'examples.terrain_following_bt.main' in task
    assert (Path('custom/terrain/terrain_heightmap.f32').is_file())
    assert (Path('custom/terrain/terrain.obj').is_file())


def test_mission_follows_continuous_route_without_settling_at_waypoints():
    value = mission()
    current = observation()
    value.reset(current)
    status = SimpleNamespace(armed=False, arming_disabled_flags=0)
    value.command(current, status, DT_S, False)
    value.command(observation(.5), status, DT_S, False)
    assert value.phase == Phase.ARMING
    status.armed = True
    value.command(observation(.6), status, DT_S, False)
    assert value.phase == Phase.TAKEOFF

    now = .7
    value.command(observation(now, clearance_m=TARGET_CLEARANCE_M), status, DT_S, False)
    now += 1.0
    value.command(observation(now, clearance_m=TARGET_CLEARANCE_M), status, DT_S, False)
    assert value.phase == Phase.TAKEOFF  # Wait for the 90-degree heading.
    now += 1.0
    value.command(observation(now, clearance_m=TARGET_CLEARANCE_M, yaw_rad=1.5707963267948966),
                  status, DT_S, False)
    now += 1.0
    value.command(observation(now, clearance_m=TARGET_CLEARANCE_M, yaw_rad=1.5707963267948966),
                  status, DT_S, False)
    assert value.phase == Phase.OUTBOUND
    assert value.heading_rad == pytest.approx(1.5707963267948966)
    assert value.outbound == ((0, 15), (0, 30), (0, 45), (0, 60), (0, 65))
    assert value.return_route == ((8, 60), (8, 45), (8, 30), (8, 15), (0, 0))
    assert value.path_xy_m[0] == (0, 0)
    assert value.path_xy_m[-1] == (0, 0)
    assert (8, 65) in value.path_xy_m
    assert RETURN_LANE_OFFSET_M == pytest.approx(2 * TURN_RADIUS_M)

    # First point is on the rising leg; later path samples advance without a dwell.
    outbound_index = value.path_xy_m.index(value.outbound[0])
    value.command(observation(now + DT_S, position=(*value.path_xy_m[outbound_index], 3.0),
                              clearance_m=TARGET_CLEARANCE_M, yaw_rad=1.5707963267948966),
                  status, DT_S, False)
    progress_before = value.path_progress_m
    value.command(observation(now + 2 * DT_S,
                              position=(*value.path_xy_m[outbound_index + 1], 3.0),
                              velocity=(0.0, 1.0, 0.0), clearance_m=TARGET_CLEARANCE_M,
                              yaw_rad=1.5707963267948966), status, DT_S, False)
    assert value.path_progress_m > progress_before
    assert value._lookahead_target() != value.outbound[0]

    now += 2 * DT_S
    for point in value.path_xy_m[outbound_index + 2:]:
        now += DT_S
        value.command(observation(now, position=(*point, 3.0),
                                  clearance_m=TARGET_CLEARANCE_M,
                                  yaw_rad=1.5707963267948966), status, DT_S, False)
    assert value.phase == Phase.RETURN
    assert [(arrival.phase, arrival.target_xy_m) for arrival in value.arrivals] == [
        *((Phase.OUTBOUND, point) for point in value.outbound),
        *((Phase.RETURN, point) for point in value.return_route),
    ]
    assert value.path_progress_m == pytest.approx(value.path_distance_m[-1])


def test_invalid_or_stale_rangefinder_aborts_without_ground_truth_fallback():
    value = mission()
    invalid = observation(valid=False)
    with pytest.raises(InvalidCommandError, match='valid rangefinder'):
        value.reset(invalid)
    value.reset(observation())
    stale = observation(.1)
    stale = DroneObservation(stale.ground_truth, SensorReadings(rangefinder=
        RangefinderReading('other', stale.ground_truth.step_index, .1, 1.5, True)))
    with pytest.raises(InvalidCommandError, match='valid rangefinder'):
        value.command(stale, SimpleNamespace(armed=False, arming_disabled_flags=0), DT_S, False)


def test_land_requires_contact_then_waits_for_betaflight_disarm():
    value = mission()
    value.reset(observation())
    value._transition(Phase.LAND, 0)
    status = SimpleNamespace(armed=True, arming_disabled_flags=0)
    value.command(observation(.1, position=(0, 0, .015), clearance_m=.01), status, DT_S, False)
    assert value.phase == Phase.LAND
    value.command(observation(.2, position=(0, 0, .015), clearance_m=.01), status, DT_S, True)
    value.command(observation(.8, position=(0, 0, .015), clearance_m=.01), status, DT_S, True)
    assert value.phase == Phase.DISARMED
    status.armed = False
    value.command(observation(1.8, position=(0, 0, .015), clearance_m=.01), status, DT_S, True)
    assert value.phase == Phase.COMPLETE


def test_unexpected_disarm_aborts_during_flight():
    value = mission()
    value.reset(observation())
    value._transition(Phase.OUTBOUND, 0)
    with pytest.raises(RuntimeError, match='disarmed during terrain flight'):
        value.command(observation(.1, clearance_m=3),
                      SimpleNamespace(armed=False, arming_disabled_flags=0), DT_S, False)


def test_startup_failure_closes_initialized_physics_environment_and_backend(monkeypatch):
    session = MagicMock()
    session.__enter__.side_effect = RuntimeError('SITL startup failed')
    physics, environment, backend = Mock(), Mock(), Mock()
    monkeypatch.setattr(demo, 'PyBulletPhysics', Mock(return_value=physics))
    monkeypatch.setattr(demo, 'DroneEnvironment', Mock(return_value=environment))
    monkeypatch.setattr(demo, 'BetaflightBackend', Mock(return_value=backend))
    monkeypatch.setattr(demo, 'SitlSession', Mock(return_value=session))
    with pytest.raises(RuntimeError, match='SITL startup failed'):
        demo.run_demo()
    physics.close.assert_called_once()
    environment.close.assert_called_once()
    backend.close.assert_called_once()


def test_cli_parser_and_headless_import():
    assert CliParser().parse([]) == DemoOptions()
    result = subprocess.run(
        [sys.executable, '-m', 'examples.terrain_following_bt.main', '--help'],
        capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert '--headless' in result.stdout
    code = """
import builtins
original = builtins.__import__
def checked(name, *args, **kwargs):
    if name.startswith(('external_render.renderer', 'zmq', 'bullet_msgs')):
        raise AssertionError('unexpected renderer import: ' + name)
    return original(name, *args, **kwargs)
builtins.__import__ = checked
import examples.terrain_following_bt.main
"""
    result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert result.stdout == result.stderr == ''
