"""GPS origin, lifecycle, FDM compatibility, and unchanged local rendering."""
from dataclasses import FrozenInstanceError, replace
from pathlib import Path
import struct
from unittest.mock import Mock

import pytest
import yaml

from bullet_drone.betaflight.fdm import pack_fdm
from bullet_drone.betaflight.firmware import GPS_FDM_MARKER, validate_gps_firmware
from bullet_drone.config import ConfigurationError, GpsSettings, SensorsSettings, YamlConfigLoader
from bullet_drone.contracts import LifecycleError
from bullet_drone.environment import DroneEnvironment
from bullet_drone.models import BarometerReading, DroneObservation, DroneState, ImuReading, SensorReadings
from bullet_drone.pose import resolve_pose
from bullet_drone.sensors import GpsSensor


ORIGIN = GpsSettings(True, 0.0, 0.0, 100.0)
ROOT = Path(__file__).resolve().parents[1]


def state(step=0, position=(0., 0., 1.), **kwargs):
    return DroneState('episode', step, step / 240, position, (0., 0., 0., 1.),
                      kwargs.pop('velocity', (1., 2., 3.)), (0., 0., 0.), **kwargs)


def observation():
    truth = state()
    return DroneObservation(truth, SensorReadings(
        ImuReading('episode', 0, 0., (0., 0., 9.81), (0., 0., 0.)),
        BarometerReading('episode', 0, 0., 100000., 12.),
        gps=GpsSensor(ORIGIN).reset(truth)))


@pytest.mark.parametrize('changes', [
    {'enabled': 1}, {'origin_latitude_deg': None}, {'origin_longitude_deg': None},
    {'origin_altitude_m': None}, {'origin_latitude_deg': 85.1},
    {'origin_latitude_deg': -85.1}, {'origin_longitude_deg': 181},
    {'origin_altitude_m': float('inf')}, {'origin_latitude_deg': float('nan')},
    {'origin_longitude_deg': True},
])
def test_invalid_settings(changes):
    with pytest.raises(ConfigurationError):
        replace(ORIGIN, **changes)


def test_yaml_defaults_and_unknown_keys(tmp_path):
    original = yaml.safe_load((ROOT / 'configs/basic.yaml').read_text())
    path = tmp_path / 'gps.yaml'
    path.write_text(yaml.safe_dump(original))
    assert not YamlConfigLoader().load(path).sensors.gps.enabled
    original['sensors']['gps'] = dict(enabled=True, origin_latitude_deg=0.,
                                      origin_longitude_deg=0., origin_altitude_m=100.)
    path.write_text(yaml.safe_dump(original))
    assert YamlConfigLoader().load(path).sensors.gps == ORIGIN
    original['sensors']['gps']['unknown'] = 1
    path.write_text(yaml.safe_dump(original))
    with pytest.raises(ConfigurationError):
        YamlConfigLoader().load(path)
    assert SensorsSettings().gps == GpsSettings()
    with pytest.raises(ConfigurationError):
        SensorsSettings(gps=None)


def test_sensor_origin_motion_and_reset():
    sensor = GpsSensor(ORIGIN)
    with pytest.raises(LifecycleError):
        sensor.sample(state())
    first = sensor.reset(state(position=(0., 0., 0.)))
    assert (first.latitude_deg, first.longitude_deg, first.altitude_m) == (0., 0., 100.)
    # Independent tabulated lengths at the equator: degrees per metre.
    moved = sensor.sample(state(1, (100., 100., 15.)))
    assert moved.latitude_deg == pytest.approx(0.000904369477, abs=1e-12)
    assert moved.longitude_deg == pytest.approx(0.000898315284, abs=1e-12)
    assert moved.altitude_m == 115.
    assert moved.velocity_enu_m_s == (1., 2., 3.)
    assert (moved.episode_id, moved.step_index, moved.simulation_time_s) == ('episode', 1, 1 / 240)
    with pytest.raises(FrozenInstanceError):
        moved.altitude_m = 0.
    with pytest.raises(ValueError):
        sensor.sample(state(1))
    next_episode = replace(state(position=(10., 20., 3.)), episode_id='next')
    with pytest.raises(LifecycleError):
        sensor.sample(next_episode)
    reset = sensor.reset(next_episode)
    assert reset.episode_id == 'next' and reset.altitude_m == 103.
    assert reset.latitude_deg > 0 and reset.longitude_deg > 0


def test_dateline_and_distance_limit():
    sensor = GpsSensor(replace(ORIGIN, origin_longitude_deg=179.9999))
    assert sensor.reset(state(position=(100., 0., 0.))).longitude_deg < -179.99
    with pytest.raises(ValueError, match='1 km'):
        sensor.sample(state(1, (1001., 0., 0.)))
    for position in ((float('nan'), 0., 0.), (0., 0., float('inf'))):
        with pytest.raises(ValueError):
            sensor.reset(state(position=position))


def test_environment_wires_gps_without_changing_pose():
    from bullet_drone.config import PoseSources
    physics = Mock()
    physics.reset.return_value = state(position=(10., 20., 3.))
    physics.step.return_value = state(1, (11., 22., 4.))
    environment = DroneEnvironment(physics, gps=GpsSensor(ORIGIN))
    initial = environment.reset()
    moved = environment.step()
    assert initial.sensors.gps.altitude_m == 103.
    assert moved.sensors.gps.altitude_m == 104.
    assert resolve_pose(moved, PoseSources()) == moved.ground_truth
    assert moved.ground_truth.position_m == (11., 22., 4.)
    environment.close()
    physics.close.assert_called_once()


def test_gps_fdm_layout_and_independent_pressure():
    sample = observation()
    packet = pack_fdm(sample, gps_enabled=True)
    assert len(packet) == 144
    values = struct.unpack('<18d', packet)
    assert values[11:14] == (1., 2., 3.)
    assert values[14:17] == (0., 0., 101.)
    assert values[17] == 100000.
    changed = replace(sample, sensors=replace(sample.sensors,
        barometer=replace(sample.sensors.barometer, pressure_pa=99000.)))
    other = struct.unpack('<18d', pack_fdm(changed, gps_enabled=True))
    assert other[14:17] == values[14:17] and other[17] == 99000.
    for gps in (None, replace(sample.sensors.gps, episode_id='stale'),
                replace(sample.sensors.gps, latitude_deg=91.),
                replace(sample.sensors.gps, altitude_m=float('nan')),
                replace(sample.sensors.gps, velocity_enu_m_s=(1., 2.)),
                replace(sample.sensors.gps, velocity_enu_m_s=(328., 0., 0.)),
                replace(sample.sensors.gps, altitude_m=21474837.),
                replace(sample.sensors.gps, latitude_deg=True)):
        with pytest.raises(ValueError):
            pack_fdm(replace(sample, sensors=replace(sample.sensors, gps=gps)), gps_enabled=True)
    with pytest.raises(ValueError, match='explicitly enabled'):
        pack_fdm(sample)


@pytest.mark.parametrize('gps', [False, True])
def test_firmware_profile_pairing(tmp_path, gps):
    binary = tmp_path / 'firmware'
    binary.write_bytes(GPS_FDM_MARKER if gps else b'upstream')
    profile = ROOT / ('configs/betaflight-gps.config' if gps else 'configs/betaflight.config')
    validate_gps_firmware(binary, profile, gps_enabled=gps)
    with pytest.raises(ValueError, match='matching'):
        validate_gps_firmware(binary, profile, gps_enabled=not gps)
    binary.write_bytes(b'upstream' if gps else GPS_FDM_MARKER)
    with pytest.raises(ValueError, match='matching'):
        validate_gps_firmware(binary, profile, gps_enabled=gps)


def test_gps_example_cli():
    from examples.altitde_hold_bt.demo_cli import CliParser
    import click
    options = CliParser().parse(['--headless', '--gps-origin', '32', '35', '100'])
    assert options.gps_settings == GpsSettings(True, 32., 35., 100.)
    assert not CliParser().parse([]).gps_settings.enabled
    with pytest.raises(click.BadParameter):
        CliParser().parse(['--gps-origin', '90', '35', '100'])
