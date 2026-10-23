from dataclasses import FrozenInstanceError, asdict, replace
from math import exp, pi
from statistics import mean, stdev
from unittest.mock import Mock
import json

import pytest
from jsonschema import Draft202012Validator
import yaml

from bullet_drone.commands import InvalidCommandError, RcInput
from bullet_drone.config import (BarometerSettings, ConfigurationError, ImuSettings,
                                 SensorsSettings, YamlConfigLoader)
from bullet_drone.contracts import LifecycleError
from bullet_drone.controllers import InternalControlSettings, PidSettings
from bullet_drone.geometry import from_rpy
from bullet_drone.environment import DroneEnvironment
from bullet_drone.models import DroneState
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.sensors import BarometerSensor, ImuSensor
from bullet_drone.sensors.barometer import ATMOSPHERIC_GRAVITY_M_S2, DRY_AIR_GAS_CONSTANT_J_KG_K
from bullet_drone.telemetry.header import SCHEMA_VERSION
from bullet_drone.telemetry.topics.sensors import SCHEMA as SENSORS_SCHEMA
from bullet_drone.simulation import RcSimulation


GRAVITY_M_S2 = (0.0, 0.0, -9.81)
DT_S = 0.01
ZERO_VECTOR = (0.0, 0.0, 0.0)
INITIAL_ALTITUDE_M = 3.0
KNOWN_ACCELERATION_M_S2 = (1.0, 2.0, 3.0)
KNOWN_GYRO_RAD_S = (1.0, 2.0, 3.0)
ACCELEROMETER_BIAS_M_S2 = (0.1, -0.2, 0.3)
GYRO_BIAS_RAD_S = (0.01, 0.02, -0.03)
NOISE_STDDEV = 2.0
PRESSURE_BIAS_PA = 10.0
NOISE_SAMPLES = 3000
NOISE_MEAN_TOLERANCE = 0.15
NOISE_STDDEV_TOLERANCE = 0.1
ROTATION_RAD = pi / 2


def state(step=0, **kwargs):
    values = dict(episode_id='episode', step_index=step, simulation_time_s=step * DT_S,
                  position_m=(0.0, 0.0, INITIAL_ALTITUDE_M), orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
                  linear_velocity_m_s=ZERO_VECTOR, angular_velocity_rad_s=ZERO_VECTOR)
    values.update(kwargs)
    return DroneState(**values)


def test_imu_stationary_acceleration_freefall_and_immutability():
    sensor = ImuSensor(GRAVITY_M_S2)
    source = state()
    initial = sensor.reset(source)
    support = tuple(-g for g in GRAVITY_M_S2)
    assert initial.specific_force_m_s2 == support
    assert sensor.sample(state(1)).specific_force_m_s2 == support
    assert sensor.sample(state(2, linear_velocity_m_s=tuple(g * DT_S for g in GRAVITY_M_S2))).specific_force_m_s2 == pytest.approx(ZERO_VECTOR)
    sensor.reset(source)
    accelerated = sensor.sample(state(1, linear_velocity_m_s=tuple(a * DT_S for a in KNOWN_ACCELERATION_M_S2)))
    assert accelerated.specific_force_m_s2 == pytest.approx(tuple(a - g for a, g in zip(KNOWN_ACCELERATION_M_S2, GRAVITY_M_S2)))
    assert source.linear_velocity_m_s == ZERO_VECTOR
    with pytest.raises(FrozenInstanceError):
        initial.specific_force_m_s2 = ZERO_VECTOR


def test_imu_rotates_force_and_gyro_into_body_axes():
    sensor = ImuSensor(GRAVITY_M_S2)
    reading = sensor.reset(state(orientation_xyzw=from_rpy((ROTATION_RAD, 0, 0)),
                                 angular_velocity_rad_s=KNOWN_GYRO_RAD_S))
    assert reading.specific_force_m_s2 == pytest.approx((0, -GRAVITY_M_S2[2], 0))
    assert reading.angular_velocity_rad_s == pytest.approx((KNOWN_GYRO_RAD_S[0], KNOWN_GYRO_RAD_S[2], -KNOWN_GYRO_RAD_S[1]))


def test_imu_bias_and_custom_gravity():
    sensor = ImuSensor(ZERO_VECTOR, ImuSettings(accelerometer_bias_m_s2=ACCELEROMETER_BIAS_M_S2,
                                              gyro_bias_rad_s=GYRO_BIAS_RAD_S))
    reading = sensor.reset(state())
    assert reading.specific_force_m_s2 == ACCELEROMETER_BIAS_M_S2
    assert reading.angular_velocity_rad_s == GYRO_BIAS_RAD_S


@pytest.mark.parametrize('altitude_m', [-20.0, 0.0, INITIAL_ALTITUDE_M, 100.0])
def test_barometer_pressure_roundtrip_and_reference(altitude_m):
    settings = BarometerSettings(reference_altitude_m=INITIAL_ALTITUDE_M)
    sensor = BarometerSensor(settings)
    reading = sensor.reset(state(position_m=(0, 0, altitude_m)))
    scale = DRY_AIR_GAS_CONSTANT_J_KG_K * settings.reference_temperature_k / ATMOSPHERIC_GRAVITY_M_S2
    assert reading.pressure_pa == pytest.approx(settings.reference_pressure_pa * exp(-(altitude_m - INITIAL_ALTITUDE_M) / scale))
    assert reading.altitude_m == pytest.approx(altitude_m, abs=1e-9)
    if altitude_m == INITIAL_ALTITUDE_M:
        assert reading.pressure_pa == settings.reference_pressure_pa
    higher = sensor.sample(state(1, position_m=(0, 0, altitude_m + INITIAL_ALTITUDE_M)))
    assert higher.pressure_pa < reading.pressure_pa


def test_pressure_bias_changes_derived_altitude():
    reference = BarometerSettings().reference_pressure_pa
    reading = BarometerSensor(BarometerSettings(pressure_bias_pa=PRESSURE_BIAS_PA)).reset(state(position_m=ZERO_VECTOR))
    assert reading.pressure_pa == reference + PRESSURE_BIAS_PA
    assert reading.altitude_m < 0


@pytest.mark.parametrize('make_sensor,field', [
    (lambda: ImuSensor(ZERO_VECTOR, ImuSettings(accelerometer_stddev_m_s2=(NOISE_STDDEV,) * 3)), 'specific_force_m_s2'),
    (lambda: ImuSensor(ZERO_VECTOR, ImuSettings(gyro_stddev_rad_s=(NOISE_STDDEV,) * 3)), 'angular_velocity_rad_s'),
    (lambda: BarometerSensor(BarometerSettings(pressure_stddev_pa=NOISE_STDDEV)), 'pressure_pa'),
])
def test_seeded_noise_statistics_independence_and_reset(make_sensor, field):
    first, second = make_sensor(), make_sensor()
    initial = first.reset(state())
    assert initial == second.reset(state())
    samples = [first.sample(state(step)) for step in range(1, NOISE_SAMPLES + 1)]
    # Another instance's stream is not advanced by the first instance.
    assert second.sample(state(1)) == samples[0]
    assert first.reset(state()) == initial
    assert first.sample(state(1)) == samples[0]
    values = [getattr(sample, field) for sample in samples]
    values = [value[0] for value in values] if isinstance(values[0], tuple) else values
    expected_mean = 0 if field != 'pressure_pa' else BarometerSensor().reset(state()).pressure_pa
    assert mean(values) == pytest.approx(expected_mean, abs=NOISE_MEAN_TOLERANCE)
    assert stdev(values) == pytest.approx(NOISE_STDDEV, abs=NOISE_STDDEV_TOLERANCE)
    if field == 'specific_force_m_s2':
        assert any(sample.specific_force_m_s2[0] != sample.specific_force_m_s2[1] for sample in samples)
        assert first.reset(state()).angular_velocity_rad_s == ZERO_VECTOR


@pytest.mark.parametrize('make_sensor', [lambda: ImuSensor(GRAVITY_M_S2), BarometerSensor,
    lambda: BarometerSensor(BarometerSettings(filter_enabled=True))])
def test_sensor_chronology_and_reset(make_sensor):
    sensor = make_sensor()
    with pytest.raises(LifecycleError):
        sensor.sample(state(1))
    sensor.reset(state())
    for invalid in (state(), state(1, simulation_time_s=0), state(1, simulation_time_s=float('nan'))):
        with pytest.raises(ValueError):
            sensor.sample(invalid)
    for invalid in (state(1, episode_id='other'), state(1, drone_id='other')):
        with pytest.raises(LifecycleError):
            sensor.sample(invalid)
    sensor.sample(state(1))
    reset = sensor.reset(state(episode_id='other'))
    assert reset.episode_id == 'other' and reset.step_index == 0
    assert sensor.sample(state(1, episode_id='other')).simulation_time_s == DT_S


@pytest.mark.parametrize('settings', [
    BarometerSettings(pressure_bias_pa=-2 * BarometerSettings().reference_pressure_pa),
])
def test_nonpositive_measured_pressure_is_rejected(settings):
    with pytest.raises(ValueError, match='pressure must be positive'):
        BarometerSensor(settings).reset(state())


@pytest.mark.parametrize('settings_type,kwargs', [
    (ImuSettings, {'enabled': 1}), (ImuSettings, {'seed': True}),
    (ImuSettings, {'accelerometer_stddev_m_s2': (-1, 0, 0)}),
    (ImuSettings, {'gyro_stddev_rad_s': (0, float('nan'), 0)}),
    (ImuSettings, {'accelerometer_bias_m_s2': (0, 0)}),
    (BarometerSettings, {'pressure_stddev_pa': -1}),
    (BarometerSettings, {'reference_pressure_pa': 0}),
    (BarometerSettings, {'reference_temperature_k': -1}),
    (BarometerSettings, {'reference_altitude_m': float('inf')}),
    (BarometerSettings, {'pressure_bias_pa': True}),
    (BarometerSettings, {'seed': None}),
    (BarometerSettings, {'filter_enabled': 1}),
    (BarometerSettings, {'filter_enabled': 'true'}),
    (BarometerSettings, {'filter_enabled': None}),
    (BarometerSettings, {'filter_time_constant_s': 0}),
    (BarometerSettings, {'filter_time_constant_s': -0.1}),
    (BarometerSettings, {'filter_time_constant_s': float('nan')}),
    (BarometerSettings, {'filter_time_constant_s': float('inf')}),
    (BarometerSettings, {'filter_time_constant_s': True}),
])
def test_invalid_python_sensor_settings(settings_type, kwargs):
    with pytest.raises(ConfigurationError):
        settings_type(**kwargs)


def test_sensor_yaml_settings_and_defaults(tmp_path):
    data = yaml.safe_load(open('configs/basic.yaml'))
    data.pop('pose_sources', None)  # Test sensor defaults independently of the demo's selected outputs.
    data.pop('sensors', None)
    path = tmp_path / 'config.yaml'
    path.write_text(yaml.safe_dump(data))
    assert YamlConfigLoader().load(path).sensors == SensorsSettings()
    data['sensors'] = {'imu': {'enabled': False, 'gyro_bias_rad_s': list(GYRO_BIAS_RAD_S)},
                       'barometer': {'pressure_stddev_pa': NOISE_STDDEV,
                                     'filter_enabled': True, 'filter_time_constant_s': 0.2}}
    path.write_text(yaml.safe_dump(data))
    loaded = YamlConfigLoader().load(path).sensors
    assert loaded.imu.enabled is False and loaded.imu.gyro_bias_rad_s == GYRO_BIAS_RAD_S
    assert loaded.barometer.pressure_stddev_pa == NOISE_STDDEV
    assert loaded.barometer.filter_enabled is True
    assert loaded.barometer.filter_time_constant_s == 0.2
    for invalid in (None, {'unknown': {}}, {'imu': None}, {'barometer': {'unknown': 0}},
                    {'imu': {'enabled': 'true'}}, {'imu': {'gyro_stddev_rad_s': [-1, 0, 0]}},
                    {'barometer': {'filter_enabled': 'true'}},
                    {'barometer': {'filter_time_constant_s': 0}}):
        data['sensors'] = invalid
        path.write_text(yaml.safe_dump(data))
        with pytest.raises(ConfigurationError):
            YamlConfigLoader().load(path)


def make_sim(config):
    control = InternalControlSettings((4, 4, 4), (2, 2, 2), (PidSettings(.05, 0, 0, .1, 0),) * 3)
    return RcSimulation(config, control)


def test_sensor_observations_and_noisy_physics_independence(config):
    noisy_settings = SensorsSettings(ImuSettings(accelerometer_stddev_m_s2=(NOISE_STDDEV,) * 3),
                                    BarometerSettings(pressure_stddev_pa=NOISE_STDDEV))
    disabled_settings = SensorsSettings(ImuSettings(enabled=False), BarometerSettings(enabled=False))
    results = []
    for settings in (SensorsSettings(), noisy_settings, disabled_settings,
                     SensorsSettings(imu=ImuSettings(enabled=False)),
                     SensorsSettings(barometer=BarometerSettings(enabled=False)),
                     YamlConfigLoader().load('configs/basic.yaml').sensors):
        sim = make_sim(replace(config, sensors=settings))
        try:
            observations = [sim.reset(), sim.step(RcInput()),
                            sim.step(sim._rc.codec.encode('angle', (.05, -.03, 0), 9.81, True))]
            reset = sim.reset()
            for observation in observations + [reset]:
                source = observation.ground_truth
                for reading, enabled in ((observation.sensors.imu, settings.imu.enabled),
                                         (observation.sensors.barometer, settings.barometer.enabled)):
                    if not enabled:
                        assert reading is None
                    else:
                        assert (reading.episode_id, reading.step_index, reading.simulation_time_s) == (source.episode_id, source.step_index, source.simulation_time_s)
                assert observation.sensors.rangefinder is None
            if settings.imu.enabled:
                assert reset.sensors.imu.specific_force_m_s2 == observations[0].sensors.imu.specific_force_m_s2
            if settings.barometer.enabled:
                assert reset.sensors.barometer.pressure_pa == observations[0].sensors.barometer.pressure_pa
                with pytest.raises(FrozenInstanceError):
                    reset.sensors.barometer.pressure_pa = 0
            results.append([replace(obs.ground_truth, episode_id='same') for obs in observations])
            if settings == SensorsSettings():
                assert observations[1].sensors.imu.specific_force_m_s2 == pytest.approx(ZERO_VECTOR)
        finally:
            sim.close()
        assert sim._environment._physics._client is None
    assert all(result == results[0] for result in results)


@pytest.mark.parametrize('stage', ['reset', 'sample'])
@pytest.mark.parametrize('failure', [ValueError('sensor failure'), InvalidCommandError('sensor failure')])
def test_sensor_failure_requires_environment_reset_and_cleanup(config, stage, failure):
    sensor = ImuSensor(GRAVITY_M_S2)
    env = DroneEnvironment(PyBulletPhysics(config), imu=sensor)
    original = getattr(sensor, stage)
    try:
        if stage == 'sample':
            env.reset()
        setattr(sensor, stage, Mock(side_effect=failure))
        with pytest.raises(ValueError, match='sensor failure'):
            env.reset() if stage == 'reset' else env.step()
        with pytest.raises(LifecycleError):
            env.step()
        setattr(sensor, stage, original)
        assert env.reset().sensors.imu is not None
        assert env.step().ground_truth.step_index == 1
    finally:
        env.close()
    assert env._physics._client is None


@pytest.mark.parametrize('filter_enabled', [False, True])
def test_sensor_payloads_match_typed_schema(config, filter_enabled):
    config = replace(config, sensors=replace(config.sensors,
                     barometer=BarometerSettings(filter_enabled=filter_enabled, pressure_stddev_pa=NOISE_STDDEV)))
    sim = make_sim(config)
    try:
        initial = sim.reset()
        stepped = sim.step(RcInput())
        for observation in (initial, stepped):
            state = observation.ground_truth
            payload = json.loads(json.dumps({
                "header": {
                    "schema_version": SCHEMA_VERSION, "drone_id": state.drone_id,
                    "episode_id": state.episode_id, "step_index": state.step_index,
                    "simulation_time_s": state.simulation_time_s,
                },
                "data": asdict(observation.sensors),
            }, allow_nan=False))
            Draft202012Validator(SENSORS_SCHEMA).validate(payload)
            for name in ('imu', 'barometer'):
                assert payload['data'][name]['step_index'] == state.step_index
                assert payload['data'][name]['simulation_time_s'] == state.simulation_time_s
            assert payload['data']['imu']['specific_force_m_s2'] == list(observation.sensors.imu.specific_force_m_s2)
            assert payload['data']['barometer']['pressure_pa'] == observation.sensors.barometer.pressure_pa
    finally:
        sim.close()


@pytest.mark.parametrize('height', [float('nan'), float('inf'), -1e308, 1e308])
def test_barometer_invalid_or_unrepresentable_height(height):
    with pytest.raises(ValueError):
        BarometerSensor().reset(state(position_m=(0, 0, height)))


def test_pressure_noise_is_used_for_altitude_inversion():
    settings = BarometerSettings(pressure_stddev_pa=NOISE_STDDEV)
    reading = BarometerSensor(settings).reset(state())
    recovered_pressure = settings.reference_pressure_pa * exp(
        -(reading.altitude_m - settings.reference_altitude_m)
        * ATMOSPHERIC_GRAVITY_M_S2
        / (DRY_AIR_GAS_CONSTANT_J_KG_K * settings.reference_temperature_k))
    assert recovered_pressure == pytest.approx(reading.pressure_pa)


def test_different_seeds_change_measurements():
    first = ImuSensor(ZERO_VECTOR, ImuSettings(gyro_stddev_rad_s=(NOISE_STDDEV,) * 3, seed=0))
    second = ImuSensor(ZERO_VECTOR, ImuSettings(gyro_stddev_rad_s=(NOISE_STDDEV,) * 3, seed=1))
    assert first.reset(state()).angular_velocity_rad_s != second.reset(state()).angular_velocity_rad_s


@pytest.mark.parametrize('times', [
    [step * DT_S for step in range(1, 51)],
    [1e-12, 0.001, 0.05, 0.17, 0.4, 1.0],
])
def test_pressure_filter_step_response_uses_measurement_intervals(times):
    settings = BarometerSettings(filter_enabled=True)
    sensor = BarometerSensor(settings)
    initial = sensor.reset(state())
    target_height_m = 20.0
    scale = DRY_AIR_GAS_CONSTANT_J_KG_K * settings.reference_temperature_k / ATMOSPHERIC_GRAVITY_M_S2
    target_pressure = settings.reference_pressure_pa * exp(-(target_height_m - settings.reference_altitude_m) / scale)
    for step, time_s in enumerate(times, 1):
        reading = sensor.sample(state(step, simulation_time_s=time_s, position_m=(0., 0., target_height_m)))
        expected_pressure = target_pressure + (initial.pressure_pa - target_pressure) * exp(-time_s / settings.filter_time_constant_s)
        assert reading.pressure_pa == pytest.approx(expected_pressure, rel=1e-12)
        recovered = settings.reference_pressure_pa * exp(-(reading.altitude_m - settings.reference_altitude_m) / scale)
        assert recovered == pytest.approx(reading.pressure_pa, rel=1e-12)
        assert INITIAL_ALTITUDE_M <= reading.altitude_m <= target_height_m


def test_disabled_filter_preserves_instantaneous_pressure_and_noise():
    settings = BarometerSettings(pressure_stddev_pa=NOISE_STDDEV)
    default = BarometerSensor(settings)
    disabled = BarometerSensor(replace(settings, filter_enabled=False, filter_time_constant_s=10.))
    assert default.reset(state()) == disabled.reset(state())
    for step in range(1, 20):
        sample = state(step, position_m=(0., 0., float(step)))
        assert default.sample(sample) == disabled.sample(sample)


def test_demo_filter_reduces_noise_and_reset_restarts_history():
    settings = YamlConfigLoader().load('configs/basic.yaml').sensors.barometer
    assert settings.filter_enabled and settings.filter_time_constant_s == 0.1
    filtered, raw = BarometerSensor(settings), BarometerSensor(replace(settings, filter_enabled=False))
    initial = filtered.reset(state())
    assert initial == raw.reset(state())
    sample_period_s = 1 / 240
    readings, raw_readings = [], []
    for step in range(1, NOISE_SAMPLES + 1):
        sample = state(step, simulation_time_s=step * sample_period_s)
        readings.append(filtered.sample(sample))
        raw_readings.append(raw.sample(sample))
    settled = readings[240:]
    noise_m = stdev(reading.altitude_m for reading in settled)
    assert 0.015 < noise_m < 0.05
    assert noise_m < stdev(reading.altitude_m for reading in raw_readings[240:]) / 4
    assert abs(mean(reading.altitude_m for reading in settled) - INITIAL_ALTITUDE_M) < 0.02
    assert filtered.reset(state()) == initial
    for step, expected in enumerate(readings[:10], 1):
        assert filtered.sample(state(step, simulation_time_s=step * sample_period_s)) == expected
    # A new episode and height seed directly, without retaining the previous low-pass state.
    replacement = state(episode_id='replacement', position_m=(0., 0., 20.))
    assert filtered.reset(replacement) == raw.reset(replacement)


@pytest.mark.parametrize('invalid_pressure', [0., -1., float('nan'), float('inf')])
def test_filter_does_not_hide_invalid_measured_pressure(monkeypatch, invalid_pressure):
    sensor = BarometerSensor(BarometerSettings(filter_enabled=True))
    sensor.reset(state())
    monkeypatch.setattr('bullet_drone.sensors.barometer.noisy', lambda *args: invalid_pressure)
    with pytest.raises(ValueError, match='pressure must be positive and finite'):
        sensor.sample(state(1))
