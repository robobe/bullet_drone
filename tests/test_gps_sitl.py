"""Opt-in checks against real pinned firmware: BULLET_TEST_GPS_SITL=1 uv run pytest tests/test_gps_sitl.py."""
import os
from pathlib import Path
import struct
from threading import Event, Thread

import pytest

from bullet_drone.betaflight.bridge import UdpSitlBridge
from bullet_drone.betaflight.fdm import pack_fdm
from bullet_drone.config import GpsSettings
from bullet_drone.models import BarometerReading, DroneObservation, DroneState, ImuReading, SensorReadings
from bullet_drone.sensors import GpsSensor
from examples.terrain_following_bt.sitl import SitlSession


ROOT = Path(__file__).resolve().parents[1]
RAW_GPS = 106
FEED_PERIOD_S = 1 / 240
pytestmark = pytest.mark.skipif(os.getenv('BULLET_TEST_GPS_SITL') != '1', reason='requires built GPS SITL and free ports')


@pytest.mark.parametrize('mavlink', [False, True])
def test_live_gps_and_independent_barometer(tmp_path, mavlink):
    prefix = 'betaflight-mavlink' if mavlink else 'betaflight'
    binary = ROOT / 'bin' / ('betaflight_mavlink_gps_SITL.elf' if mavlink else 'betaflight_gps_SITL.elf')
    # Isolate barometer output from intentional GPS-altitude changes during this check.
    profile = tmp_path / 'gps.config'
    profile.write_text((ROOT / f'configs/{prefix}-gps.config').read_text() +
                       '\nset altitude_source = BARO_ONLY\n' +
                       ('set mavlink_extra2_rate = 10\n' if mavlink else '') + 'save\n')
    settings = GpsSettings(True, 32., 35., 100.)
    expected = GpsSensor(settings)
    for spawn in ((10., 20., 3.), (30., 40., 8.)):
        position, velocity, pressure = spawn, (0., 0., 0.), 101325.
        stop = Event()
        errors = []
        bridge = UdpSitlBridge()
        with SitlSession(binary, profile_path=profile, gps_enabled=True,
                         extra_ports=(5762,) if mavlink else ()) as session:
            bridge.open()
            connection = None
            def feed():
                step = 0
                try:
                    while not stop.is_set():
                        point, speed, measured_pressure = position, velocity, pressure
                        truth = DroneState('gps-test', step, step * FEED_PERIOD_S, point,
                                           (0., 0., 0., 1.), speed, (0., 0., 0.))
                        gps = expected.reset(truth)
                        sensors = SensorReadings(
                            ImuReading(truth.episode_id, step, truth.simulation_time_s, (0., 0., 9.81), (0., 0., 0.)),
                            BarometerReading(truth.episode_id, step, truth.simulation_time_s, measured_pressure, 0.),
                            gps=gps)
                        bridge.send_fdm(pack_fdm(DroneObservation(truth, sensors), gps_enabled=True))
                        bridge.receive_motor_packets()
                        step += 1
                        stop.wait(FEED_PERIOD_S)
                except BaseException as error:
                    errors.append(error)
                    stop.set()
            feeder = Thread(target=feed)
            feeder.start()
            def read_gps():
                return struct.unpack_from('<BBiiHHH', session.client.request(RAW_GPS).payload)
            def settle(seconds=1.5):
                assert not stop.wait(seconds), errors
                session.check_running()
            try:
                settle(3.)
                fix, satellites, lat, lon, alt, speed, course = read_gps()
                initial = GpsSensor(settings).reset(DroneState('ref', 0, 0., spawn, (0., 0., 0., 1.), velocity, velocity))
                assert fix and satellites == 12
                assert lat / 1e7 == pytest.approx(initial.latitude_deg, abs=2e-7)
                assert lon / 1e7 == pytest.approx(initial.longitude_deg, abs=2e-7)
                assert alt == int(initial.altitude_m)
                if mavlink:
                    from pymavlink import mavutil
                    connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
                    assert connection.wait_heartbeat(timeout=5.) is not None
                    for _ in range(100):
                        if session.commands.read_status().arming_disabled_flags == 0:
                            break
                        settle(0.1)
                    assert session.commands.read_status().arming_disabled_flags == 0
                    session.client.rc.update((1500, 1500, 1000, 1500, 2000, 1000) + (1000,) * 12)
                    settle(1.)
                    assert session.commands.read_status().armed
                    def hud_altitude():
                        for _ in range(1000):
                            if connection.recv_match(blocking=False) is None:
                                break
                        message = connection.recv_match(type='VFR_HUD', blocking=True, timeout=3.)
                        assert message is not None
                        return message.alt
                    assert abs(hud_altitude()) < 0.5
                baseline = session.commands.read_altitude().altitude_m
                position, velocity = (spawn[0] + 10., spawn[1], spawn[2]), (2., 0., 0.)
                settle()
                east = read_gps()
                assert east[3] > lon and east[5] == 200 and abs(east[6] - 900) <= 1
                position, velocity = (spawn[0] + 10., spawn[1] + 10., spawn[2]), (0., 2., 0.)
                settle()
                north = read_gps()
                assert north[2] > lat and north[5] == 200 and north[6] <= 1
                # GPS altitude changes without changing pressure must leave barometer altitude fixed.
                velocity = (0., 0., 0.)
                position = (*position[:2], spawn[2] + 20.)
                settle()
                assert read_gps()[4] == int(initial.altitude_m + 20.)
                before = session.commands.read_altitude().altitude_m
                assert abs(before - baseline) < 0.5
                pressure = 101225.
                settle(3.)
                assert read_gps()[4] == int(initial.altitude_m + 20.)
                assert session.commands.read_altitude().altitude_m - before > 6.
                if mavlink:
                    assert 6. < hud_altitude() < 11.
            finally:
                stop.set()
                feeder.join(2.)
                bridge.close()
                if connection is not None:
                    connection.close()
            assert not feeder.is_alive() and not errors
