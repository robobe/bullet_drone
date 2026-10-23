"""Sensor-driven attitude math, lifecycle and deterministic physics integration."""
from dataclasses import replace
from math import hypot

import pytest

from bullet_drone.attitude import AttitudeEstimator
from bullet_drone.commands import RcInput
from bullet_drone.config import (
    AttitudeEstimatorSettings, BarometerSettings, ConfigurationError, ImuSettings,
    PoseSources, SensorsSettings, YamlConfigLoader,
)
from bullet_drone.contracts import LifecycleError
from bullet_drone.environment import DroneEnvironment
from bullet_drone.geometry import attitude_error, from_rpy, world_to_body, yaw_from_quaternion
from bullet_drone.models import ImuReading
from bullet_drone.pose import resolve_pose
from bullet_drone.sensors import ImuSensor
from bullet_drone.simulation import RcSimulation
from examples.position_square.demo import FLIGHT_CONTROL_SETTINGS


GRAVITY = (0., 0., -9.81)
DT = 1 / 240


def reading(step, *, gyro=(0., 0., 0.), force=(0., 0., 9.81)):
    return ImuReading('episode', step, step * DT, force, gyro)


def distance(a, b):
    return hypot(*attitude_error(a, b))


@pytest.mark.parametrize('rpy', [(0., 0., 0.), (.4, -.3, 1.2)])
def test_stationary_tilt_and_known_initial_heading(rpy):
    q = from_rpy(rpy)
    force = world_to_body((0., 0., 9.81), q)
    estimator = AttitudeEstimator(GRAVITY)
    estimator.reset(reading(0, force=force), q)
    for step in range(1, 241):
        estimate = estimator.update(reading(step, force=force))
    assert distance(estimate.orientation_xyzw, q) < 1e-12
    assert (estimate.episode_id, estimate.step_index, estimate.simulation_time_s) == ('episode', 240, 1.)


@pytest.mark.parametrize('axis', [0, 1, 2])
def test_body_rate_rotation_and_unit_quaternion(axis):
    estimator = AttitudeEstimator(GRAVITY, AttitudeEstimatorSettings(kp_s_inv=0, ki_s_inv2=0))
    estimator.reset(reading(0), from_rpy((0., 0., 0.)))
    rates = tuple(1. if i == axis else 0. for i in range(3))
    for step in range(1, 481):
        estimate = estimator.update(reading(step, gyro=rates, force=(0., 0., 0.)))
    expected = from_rpy(tuple(2. if i == axis else 0. for i in range(3)))
    assert distance(estimate.orientation_xyzw, expected) < 1e-12
    assert hypot(*estimate.orientation_xyzw) == pytest.approx(1.)


def test_accelerometer_corrects_tilt_and_estimates_gyro_bias():
    estimator = AttitudeEstimator(GRAVITY, AttitudeEstimatorSettings(ki_s_inv2=.5))
    estimator.reset(reading(0), from_rpy((.3, -.2, .7)))
    for step in range(1, 12001):
        estimate = estimator.update(reading(step, gyro=(.1, 0., 0.)))
    # A constant accelerometer reference removes roll/pitch drift, but supplies no heading.
    up = world_to_body((0., 0., 1.), estimate.orientation_xyzw)
    assert up == pytest.approx((0., 0., 1.), abs=1e-5)
    assert estimator._bias == pytest.approx((.1, 0., 0.), abs=1e-5)


@pytest.mark.parametrize('force', [(0., 0., 0.), (0., 0., 20.), (20., 0., 9.81)])
def test_invalid_gravity_magnitude_uses_gyro_only(force):
    estimator = AttitudeEstimator(GRAVITY)
    estimator.reset(reading(0), from_rpy((.3, 0., 0.)))
    for step in range(1, 241):
        estimate = estimator.update(reading(step, gyro=(.1, 0., 0.), force=force))
    assert distance(estimate.orientation_xyzw, from_rpy((.4, 0., 0.))) < 1e-12
    assert estimator._bias == (0., 0., 0.)


def test_yaw_drift_is_not_corrected_with_gravity_and_reset_clears_bias():
    estimator = AttitudeEstimator(GRAVITY)
    seed = from_rpy((0., 0., .7))
    estimator.reset(reading(0), seed)
    for step in range(1, 241):
        estimate = estimator.update(reading(step, gyro=(0., 0., .1)))
    assert yaw_from_quaternion(estimate.orientation_xyzw) == pytest.approx(.8)
    assert estimator.reset(reading(0), seed).orientation_xyzw == pytest.approx(seed)
    assert estimator._bias == (0., 0., 0.)


def test_timestamp_episode_and_numeric_validation():
    estimator = AttitudeEstimator(GRAVITY)
    with pytest.raises(LifecycleError):
        estimator.update(reading(1))
    estimator.reset(reading(0), (0., 0., 0., 1.))
    for sample in (reading(0), replace(reading(1), simulation_time_s=0.),
                   reading(1, gyro=(float('nan'), 0., 0.)),
                   replace(reading(1), simulation_time_s=float('inf'))):
        with pytest.raises(ValueError):
            estimator.update(sample)
    with pytest.raises(LifecycleError):
        estimator.update(replace(reading(1), episode_id='other'))
    assert estimator.update(reading(1)).step_index == 1


@pytest.mark.parametrize('field,value', [('kp_s_inv', -1), ('ki_s_inv2', float('nan')),
    ('kp_s_inv', True), ('acceleration_rejection_fraction', 0),
    ('acceleration_rejection_fraction', 1)])
def test_invalid_filter_settings(field, value):
    with pytest.raises(ConfigurationError):
        AttitudeEstimatorSettings(**{field: value})


@pytest.mark.parametrize('sources,sensors', [
    (PoseSources(attitude='imu'), SensorsSettings(imu=ImuSettings(enabled=False))),
    (PoseSources(altitude='barometer'), SensorsSettings(barometer=BarometerSettings(enabled=False))),
])
def test_disabled_required_sensor_rejected(config, sources, sensors):
    with pytest.raises(ConfigurationError):
        replace(config, pose_sources=sources, sensors=sensors)


def test_yaml_sources_and_defaults(config, tmp_path):
    import yaml
    from dataclasses import asdict
    path = tmp_path / 'physics.yaml'
    data = asdict(config)
    data['pose_sources'] = dict(altitude='barometer', attitude='imu')
    data['attitude_estimator'] = dict(kp_s_inv=3., ki_s_inv2=.1)
    path.write_text(yaml.safe_dump(data))
    loaded = YamlConfigLoader().load(path)
    assert loaded.pose_sources == PoseSources('barometer', 'imu')
    assert loaded.attitude_estimator == AttitudeEstimatorSettings(3., .1)
    for section, value in [('pose_sources', {'attitude': 'unknown'}), ('pose_sources', None),
                           ('pose_sources', {'position': 'imu'}), ('attitude_estimator', {'unknown': 1})]:
        invalid = dict(data, **{section: value})
        path.write_text(yaml.safe_dump(invalid))
        with pytest.raises(ConfigurationError):
            YamlConfigLoader().load(path)
    assert config.pose_sources == PoseSources()
    with pytest.raises(ConfigurationError):
        replace(config, pose_sources=PoseSources(attitude='imu'),
                simulation=replace(config.simulation, gravity_m_s2=(0., 0., 0.)))


@pytest.mark.parametrize('noise', ['accelerometer', 'gyro', 'barometer'])
def test_noise_propagates_repeatably_without_changing_physics(config, noise):
    sources = PoseSources('barometer', 'imu')
    imu = ImuSettings(accelerometer_stddev_m_s2=(.2,)*3 if noise == 'accelerometer' else (0.,)*3,
                      gyro_stddev_rad_s=(.1,)*3 if noise == 'gyro' else (0.,)*3)
    sensors = SensorsSettings(imu, BarometerSettings(pressure_stddev_pa=3. if noise == 'barometer' else 0.))
    runs = []
    for sensor_settings in (SensorsSettings(), sensors, sensors):
        sim = RcSimulation(replace(config, pose_sources=sources, sensors=sensor_settings), FLIGHT_CONTROL_SETTINGS)
        try:
            # Disarmed ground contact supplies an observable gravity reference.
            observations = [sim.reset()]
            observations.extend(sim.step(RcInput()) for _ in range(240))
            reset = sim.reset()
            assert reset.attitude_estimate.orientation_xyzw == observations[0].attitude_estimate.orientation_xyzw
            repeated = [sim.step(RcInput()) for _ in range(240)]
            assert [o.attitude_estimate.orientation_xyzw for o in repeated] == [o.attitude_estimate.orientation_xyzw for o in observations[1:]]
            assert [o.sensors.barometer.altitude_m for o in repeated] == [o.sensors.barometer.altitude_m for o in observations[1:]]
            runs.append(observations)
        finally:
            sim.close()
    baseline, noisy, repeated = runs
    assert [replace(o.ground_truth, episode_id='same') for o in baseline] == [replace(o.ground_truth, episode_id='same') for o in noisy]
    def values(observations):
        return [(resolve_pose(o, sources).position_m, resolve_pose(o, sources).orientation_xyzw) for o in observations]
    assert values(noisy) == values(repeated)
    assert values(noisy) != values(baseline)


def test_missing_or_mismatched_pose_source_never_falls_back(config):
    sim = RcSimulation(config, FLIGHT_CONTROL_SETTINGS)
    try:
        observation = sim.reset()
        assert observation.attitude_estimate is None
        assert resolve_pose(observation, PoseSources()) is observation.ground_truth
        with pytest.raises(ValueError):
            resolve_pose(observation, PoseSources(attitude='imu'))
        mismatched = replace(observation, sensors=replace(observation.sensors,
                              barometer=replace(observation.sensors.barometer, step_index=1)))
        with pytest.raises(ValueError):
            resolve_pose(mismatched, PoseSources(altitude='barometer'))
    finally:
        sim.close()


def test_estimator_failure_invalidates_environment_until_reset(config):
    class Estimator(AttitudeEstimator):
        def update(self, reading):
            raise ValueError('injected filter failure')
    from bullet_drone.physics import PyBulletPhysics
    env = DroneEnvironment(PyBulletPhysics(config), imu=ImuSensor(GRAVITY), attitude_estimator=Estimator(GRAVITY))
    try:
        env.reset()
        with pytest.raises(ValueError, match='injected filter failure'):
            env.step()
        with pytest.raises(LifecycleError):
            env.step()
        assert env.reset().attitude_estimate.step_index == 0
    finally:
        env.close()
