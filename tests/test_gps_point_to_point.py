"""Navigation and telemetry checks independent of a running firmware process."""
from types import SimpleNamespace as Message
from math import dist, hypot
import ast
from pathlib import Path

import pytest

from bullet_drone.geometry import from_rpy
from bullet_drone.models import DroneState
from bullet_drone.sensors import GpsSensor
from bullet_drone.config import GpsSettings
from examples.gps_point_to_point_mavlink.navigation import GeographicFrame, line_velocity
from examples.gps_point_to_point_mavlink.telemetry import Telemetry
from examples.gps_point_to_point_mavlink.mission import Mission


def receiver():
    value = Telemetry(GeographicFrame(32., 35., 100.))
    value.latest = {
        'HEARTBEAT': (Message(), 10.),
        'GPS_RAW_INT': (Message(fix_type=3, satellites_visible=12), 10.),
        'GLOBAL_POSITION_INT': (Message(lat=320000000, lon=350000000, alt=110000, vx=100, vy=200, vz=-300), 10.),
        # Captured pinned firmware signs: roll matches truth, pitch is negated, yaw matches truth.
        'ATTITUDE': (Message(roll=.1, pitch=-.2, yaw=.3), 10.),
    }
    return value


def test_pinned_firmware_units_and_attitude_signs():
    feedback = receiver().feedback(10.1)
    assert feedback.geographic_position == pytest.approx((32., 35., 110.))
    assert feedback.position_m == pytest.approx((0., 0., 10.))
    assert feedback.velocity_m_s == (2., 1., 3.)
    assert feedback.attitude.orientation_xyzw == pytest.approx(from_rpy((.1, .2, .3)))
    assert feedback.yaw_rad == .3


@pytest.mark.parametrize('kind', ['HEARTBEAT', 'GPS_RAW_INT', 'GLOBAL_POSITION_INT', 'ATTITUDE'])
def test_missing_and_stale_feedback_rejected(kind):
    value = receiver()
    message, _ = value.latest[kind]
    value.latest[kind] = message, 0.
    with pytest.raises(RuntimeError, match='Stale'):
        value.feedback(10.1)
    del value.latest[kind]
    with pytest.raises(RuntimeError, match='Missing'):
        value.feedback(10.1)


def test_invalid_fix_and_attitude_rejected():
    value = receiver()
    value.latest['GPS_RAW_INT'][0].fix_type = 1
    with pytest.raises(RuntimeError, match='fix'):
        value.feedback(10.1)
    value = receiver()
    value.latest['ATTITUDE'][0].roll = float('nan')
    with pytest.raises(RuntimeError, match='attitude'):
        value.feedback(10.1)


def test_coordinates_match_sensor_and_route_is_400_m():
    frame = GeographicFrame(32., 35., 100.)
    truth = DroneState('test', 0, 0., (200., 10., 20.), (0., 0., 0., 1.), (0.,)*3, (0.,)*3)
    gps = GpsSensor(GpsSettings(True, 32., 35., 100.)).reset(truth)
    assert frame.local(gps.latitude_deg, gps.longitude_deg, gps.altitude_m) == pytest.approx(truth.position_m)
    from examples.gps_point_to_point_mavlink.main import START_GPS, DESTINATION_GPS
    start, end = [frame.local(*point, 100.) for point in (START_GPS, DESTINATION_GPS)]
    assert dist(start, end) == pytest.approx(400.)
    with pytest.raises(ValueError, match='1 km'):
        frame.local(33., 35., 100.)


def test_line_guidance_cross_track_and_braking():
    start, end = (0., 0.), (400., 0.)
    assert line_velocity(start, end, (100., 0.)) == (5., 0.)
    vx, vy = line_velocity(start, end, (100., 2.))
    assert vx > 0 and vy < 0
    assert hypot(vx, vy) == pytest.approx(5.)
    assert 0 < line_velocity(start, end, (399.9, 0.))[0] < 1
    assert line_velocity(start, end, end) == (0., 0.)
    assert line_velocity(start, end, (401., 0.))[0] < 0


def test_landing_requires_stable_destination_and_disarm():
    mission = Mission((0., 0.), (400., 0.), 10., .015, 1., 9.81, 40.)
    status = Message(armed=True, arming_disabled_flags=0)
    feedback = receiver().feedback(10.1)
    from dataclasses import replace
    feedback = replace(feedback, position_m=(400., 0., .015), velocity_m_s=(0.,)*3)
    mission.phase = 'land'
    mission.command(feedback, status, 1., .01)
    channels = mission.command(feedback, status, 2.1, .01)
    assert mission.phase == 'disarmed' and channels[4] == 1000
    status.armed = False
    mission.command(feedback, status, 2.2, .01)
    mission.command(feedback, status, 3.3, .01)
    assert mission.phase == 'complete'


def test_no_other_example_imports_or_ground_truth_in_controller():
    folder = Path('examples/gps_point_to_point_mavlink')
    for path in folder.rglob('*.py'):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module and node.module.startswith('examples.'):
                assert node.module.startswith('examples.gps_point_to_point_mavlink')
    assert 'ground_truth' not in (folder / 'mission.py').read_text()
    assert 'ground_truth' not in (folder / 'telemetry.py').read_text()


def test_corridor_rejects_obstacle_and_missing_surface():
    from examples.gps_point_to_point_mavlink.demo import check_corridor, surface_height
    class FlatMap:
        def ray_surface(self, origin, direction, length):
            return Message(distance_m=500.)
    assert check_corridor(FlatMap(), (0., 0.), (10., 0.), 10.) == 10.
    with pytest.raises(ValueError, match='Unsafe'):
        check_corridor(FlatMap(), (0., 0.), (10., 0.), 2.)
    class MissingMap:
        def ray_surface(self, *args):
            return None
    with pytest.raises(ValueError, match='No mapped terrain'):
        surface_height(MissingMap(), 0., 0.)


def test_profile_uses_actual_pinned_cli_key():
    profile = Path('examples/gps_point_to_point_mavlink/firmware.config').read_text()
    assert 'set mavlink_pos_rate = 20' in profile
    assert 'set mavlink_position_rate' not in profile


def test_real_sitl_mission_when_requested():
    import os
    if os.getenv('BULLET_TEST_GPS_POINT_TO_POINT') != '1':
        pytest.skip('requires GPS MAVLink binary, free SITL ports and a full paced flight')
    from examples.gps_point_to_point_mavlink.demo import run_demo
    from examples.gps_point_to_point_mavlink.main import (
        GPS_ORIGIN, START_GPS, DESTINATION_GPS, TERRAIN_PATH, PROJECT_ROOT,
    )
    result = run_demo(origin=GPS_ORIGIN, start_gps=START_GPS, end_gps=DESTINATION_GPS,
                      terrain_path=TERRAIN_PATH,
                      sitl_binary=PROJECT_ROOT / 'bin/betaflight_mavlink_gps_SITL.elf')
    assert [phase for phase, _ in result.transitions] == ['arming', 'takeoff', 'cruise', 'land', 'disarmed', 'complete']
    assert result.landing_error_m < .5
    assert result.max_cross_track_m < .5
    assert result.minimum_clearance_m > 3.
    assert result.contact and not result.armed


def test_rejected_firmware_profile_cleans_up(tmp_path, monkeypatch):
    from unittest.mock import MagicMock
    from examples.gps_point_to_point_mavlink import sitl
    binary = tmp_path / 'firmware'
    binary.write_text("#!/bin/sh\n# BULLET_GPS_FDM_V1\necho '###ERROR IN set: INVALID NAME###'\n")
    binary.chmod(0o755)
    monkeypatch.setattr(sitl.socket, 'socket', MagicMock())
    session = sitl.SitlSession(binary)
    with pytest.raises(ValueError, match='rejected'):
        with session:
            pytest.fail('Rejected configuration must not launch SITL')
    assert session.process is None
    assert session.log.closed
    assert not Path(session.work.name).exists()


def test_default_route_clears_mesh_and_has_a_flat_landing_site():
    from examples.gps_point_to_point_mavlink import demo
    from examples.gps_point_to_point_mavlink.main import GPS_ORIGIN, START_GPS, DESTINATION_GPS, TERRAIN_PATH
    from bullet_drone.config import SimulationConfig, SimulationSettings, DroneSettings, TerrainSettings
    from bullet_drone.physics import PyBulletPhysics
    frame = GeographicFrame(*GPS_ORIGIN)
    start, end = [frame.local(*point, GPS_ORIGIN[2])[:2] for point in (START_GPS, DESTINATION_GPS)]
    config = SimulationConfig(
        SimulationSettings(demo.TIMESTEP_S, 1, demo.GRAVITY_VECTOR_M_S2, gui=False),
        DroneSettings(1., (0., 0., 100.)),
        terrain=TerrainSettings(heightmap_path=str(TERRAIN_PATH), world_size_m=500.,
                                samples_x=121, samples_z=121, origin_m=(-250., -250.)))
    physics = PyBulletPhysics(config)
    try:
        physics.reset()
        cruise = demo.surface_height(physics, *start) + demo.TAKEOFF_HEIGHT_M
        assert demo.check_corridor(physics, start, end, cruise) > 3.
        demo.check_landing_patch(physics, end)
        # The original west endpoint caused a real post-disarm slide on its slope.
        with pytest.raises(ValueError, match='too steep'):
            demo.check_landing_patch(physics, (-200., 0.))
    finally:
        physics.close()


def test_cruise_velocity_ramps_to_faster_speed_and_resets():
    mission = Mission((0., 0.), (400., 0.), 10., .015, 1., 9.81, 40.)
    assert hypot(*mission.cruise_velocity((100., 0.), .1)) == pytest.approx(.1)
    previous = mission.cruise_velocity((100., 0.), .1)
    for _ in range(60):
        current = mission.cruise_velocity((100., 0.), .1)
        assert dist(previous, current) <= .1 + 1e-9
        previous = current
    assert current == pytest.approx((5., 0.))
    current = mission.cruise_velocity((100., 2.), .1)
    assert dist(previous, current) <= .1 + 1e-9
    mission.transition('cruise', 1.)
    assert hypot(*mission.cruise_velocity((100., 0.), .1)) == pytest.approx(.1)
