"""Datasheet density conventions, discrete filtering and residual bias dynamics."""
from dataclasses import replace
from math import cos, pi, sin, sqrt
from random import Random
from statistics import mean, stdev

import pytest

from bullet_drone.config import ConfigurationError, ImuSettings, YamlConfigLoader
from bullet_drone.contracts import LifecycleError
from bullet_drone.models import DroneState
from bullet_drone.sensors import ImuSensor


ZERO = (0., 0., 0.)
SAMPLE_COUNT = 12000
ACCEL_DENSITY = (65e-6 * 9.80665, 65e-6 * 9.80665, 70e-6 * 9.80665)
GYRO_DENSITY = (.0028 * pi / 180,) * 3


def state(step=0, time=0., *, velocity=ZERO, rates=ZERO, episode='episode'):
    return DroneState(episode, step, time, ZERO, (0., 0., 0., 1.), velocity, rates)


def density_settings(**kwargs):
    return ImuSettings(noise_model='density', accelerometer_noise_density_m_s2_sqrt_hz=ACCEL_DENSITY,
                       gyro_noise_density_rad_s_sqrt_hz=GYRO_DENSITY, **kwargs)


def correlation(first, second):
    a, b = mean(first), mean(second)
    numerator = sum((x - a) * (y - b) for x, y in zip(first, second))
    denominator = sqrt(sum((x - a)**2 for x in first) * sum((y - b)**2 for y in second))
    return numerator / denominator


def test_legacy_seed_sequence_is_unchanged():
    settings = ImuSettings(accelerometer_stddev_m_s2=(.01, .02, .03),
                           gyro_stddev_rad_s=(.001, .002, .003),
                           accelerometer_bias_m_s2=(.1, .2, .3), gyro_bias_rad_s=(.01, .02, .03))
    sensor = ImuSensor((0., 0., -9.81), settings)
    rng = Random(settings.seed)
    for step in range(12):
        sample = state(step, step / 240, rates=(.1, .2, .3))
        reading = sensor.reset(sample) if step == 0 else sensor.sample(sample)
        force = tuple(value + bias + rng.gauss(0., sigma) for value, bias, sigma in
                      zip((0., 0., 9.81), settings.accelerometer_bias_m_s2, settings.accelerometer_stddev_m_s2))
        rates = tuple(value + bias + rng.gauss(0., sigma) for value, bias, sigma in
                      zip((.1, .2, .3), settings.gyro_bias_rad_s, settings.gyro_stddev_rad_s))
        assert reading.specific_force_m_s2 == force
        assert reading.angular_velocity_rad_s == rates


@pytest.mark.parametrize('rate', [240, 480, 960])
@pytest.mark.parametrize('filtered', [False, True])
def test_density_statistics_filter_variance_and_axis_independence(rate, filtered):
    settings = density_settings(filter_enabled=filtered)
    sensor = ImuSensor(ZERO, settings, sample_period_s=1/rate)
    sensor.reset(state())
    readings = [sensor.sample(state(step, step/rate)) for step in range(1, SAMPLE_COUNT + 201)]
    readings = readings[200:]
    for field, densities, cutoff in [('specific_force_m_s2', ACCEL_DENSITY, settings.accelerometer_cutoff_hz),
                                      ('angular_velocity_rad_s', GYRO_DENSITY, settings.gyro_cutoff_hz)]:
        axes = [[getattr(reading, field)[axis] for reading in readings] for axis in range(3)]
        # Solve the -3 dB condition for a one-pole filter independently of its implementation.
        c = 2 - cos(2*pi*cutoff/rate)
        pole = c - sqrt(c*c - 1)
        fraction = (1-pole)/(1+pole) if filtered else 1.
        for values, density in zip(axes, densities):
            expected = density * sqrt(rate/2 * fraction)
            assert stdev(values) == pytest.approx(expected, rel=.08)
            assert abs(mean(values)) < expected * .1
        assert abs(correlation(axes[0], axes[1])) < .08
        assert correlation(axes[0][:-1], axes[0][1:]) == pytest.approx(pole if filtered else 0., abs=.035)


@pytest.mark.parametrize('channel,cutoff', [('accelerometer', 30.), ('gyro', 60.)])
def test_filter_has_selected_digital_half_power_cutoff(channel, cutoff):
    rate = 240
    sensor = ImuSensor(ZERO, ImuSettings(filter_enabled=True), sample_period_s=1/rate)
    sensor.reset(state())
    outputs, inputs = [], []
    velocity = 0.
    for step in range(1, 1201):
        signal = sin(2*pi*cutoff*step/rate)
        velocity += signal/rate
        reading = sensor.sample(state(step, step/rate, velocity=(velocity, 0., 0.), rates=(signal, 0., 0.)))
        if step > 200:
            inputs.append(signal)
            outputs.append((reading.specific_force_m_s2 if channel == 'accelerometer' else reading.angular_velocity_rad_s)[0])
    assert stdev(outputs)/stdev(inputs) == pytest.approx(1/sqrt(2), abs=.005)


def test_reset_uses_nominal_interval_then_actual_intervals(monkeypatch):
    sensor = ImuSensor(ZERO, density_settings(), sample_period_s=.01)
    monkeypatch.setattr(sensor._rng, 'gauss', lambda center, sigma: sigma)
    initial = sensor.reset(state())
    second = sensor.sample(state(1, .02))
    assert initial.specific_force_m_s2 == pytest.approx(tuple(d/sqrt(.02) for d in ACCEL_DENSITY))
    assert second.specific_force_m_s2 == pytest.approx(tuple(d/sqrt(.04) for d in ACCEL_DENSITY))


def test_random_walk_variance_tracks_elapsed_time_and_reset_repeats():
    settings = ImuSettings(accelerometer_bias_random_walk_m_s2_sqrt_s=(.02,)*3,
                           gyro_bias_random_walk_rad_s_sqrt_s=(.001,)*3,
                           gyro_bias_rad_s=(.1,)*3)
    early_force, late_force, early_rate, late_rate = [], [], [], []
    for seed in range(1000):
        sensor = ImuSensor(ZERO, replace(settings, seed=seed))
        initial = sensor.reset(state())
        early = sensor.sample(state(1, .25))
        late = sensor.sample(state(2, 1.))
        early_force.append(early.specific_force_m_s2[0])
        late_force.append(late.specific_force_m_s2[0])
        early_rate.append(early.angular_velocity_rad_s[0])
        late_rate.append(late.angular_velocity_rad_s[0])
        assert sensor.reset(state()) == initial
        assert sensor.sample(state(1, .25)) == early
        assert sensor.sample(state(2, 1.)) == late
    assert stdev(early_force) == pytest.approx(.02*sqrt(.25), rel=.1)
    assert stdev(late_force) == pytest.approx(.02, rel=.1)
    assert stdev(early_rate) == pytest.approx(.001*sqrt(.25), rel=.1)
    assert stdev(late_rate) == pytest.approx(.001, rel=.1)
    assert mean(late_rate) == pytest.approx(.1, abs=.0001)


def test_density_filter_reset_identity_and_bad_interval():
    settings = density_settings(filter_enabled=True, gyro_bias_random_walk_rad_s_sqrt_s=(.001,)*3)
    sensor = ImuSensor(ZERO, settings, sample_period_s=1/240)
    initial = sensor.reset(state())
    first = sensor.sample(state(1, 1/240))
    for sample, error in [(state(1, 1/240), ValueError), (state(2, 1/240), ValueError),
                           (state(2, .1), ConfigurationError), (state(2, 2/240, episode='new'), LifecycleError)]:
        with pytest.raises(error):
            sensor.sample(sample)
    assert sensor.reset(state()) == initial
    assert sensor.sample(state(1, 1/240)) == first
    other = ImuSensor(ZERO, settings, sample_period_s=1/240)
    assert other.reset(state(episode='new')).specific_force_m_s2 == initial.specific_force_m_s2


@pytest.mark.parametrize('kwargs', [
    {'noise_model': 'unknown'}, {'filter_enabled': 1}, {'filter_enabled': 'true'},
    {'noise_model': 'density', 'gyro_stddev_rad_s': (.001,)*3},
    {'noise_model': 'per_sample', 'gyro_noise_density_rad_s_sqrt_hz': (.001,)*3},
    {'noise_model': 'density', 'accelerometer_noise_density_m_s2_sqrt_hz': (-1., 0., 0.)},
    {'gyro_noise_density_rad_s_sqrt_hz': (float('nan'),)*3},
    {'accelerometer_bias_random_walk_m_s2_sqrt_s': (-1., 0., 0.)},
    {'gyro_bias_random_walk_rad_s_sqrt_s': (True, 0., 0.)},
    {'accelerometer_cutoff_hz': 0}, {'gyro_cutoff_hz': float('inf')},
])
def test_invalid_settings_rejected(kwargs):
    with pytest.raises(ConfigurationError):
        ImuSettings(**kwargs)


def test_period_and_nyquist_are_validated_before_simulation_resources(config):
    for settings in [density_settings(), ImuSettings(filter_enabled=True)]:
        with pytest.raises(ConfigurationError, match='sample_period_s'):
            ImuSensor(ZERO, settings)
    for period in [0, -1., float('nan'), True]:
        with pytest.raises(ConfigurationError):
            ImuSensor(ZERO, density_settings(), sample_period_s=period)
    for cutoff in [120., 121.]:
        settings = ImuSettings(filter_enabled=True, gyro_cutoff_hz=cutoff)
        with pytest.raises(ConfigurationError, match='Nyquist'):
            ImuSensor(ZERO, settings, sample_period_s=1/240)
        with pytest.raises(ConfigurationError, match='Nyquist'):
            replace(config, sensors=replace(config.sensors, imu=settings))


def test_yaml_profile_matches_datasheet_and_calibrated_defaults():
    settings = YamlConfigLoader().load('configs/basic.yaml').sensors.imu
    assert settings.noise_model == 'density' and settings.filter_enabled
    assert settings.accelerometer_noise_density_m_s2_sqrt_hz == pytest.approx(ACCEL_DENSITY)
    assert settings.gyro_noise_density_rad_s_sqrt_hz == pytest.approx(GYRO_DENSITY)
    assert settings.accelerometer_bias_m_s2 == settings.gyro_bias_rad_s == ZERO
    assert settings.accelerometer_bias_random_walk_m_s2_sqrt_s == settings.gyro_bias_random_walk_rad_s_sqrt_s == ZERO
    assert not ImuSettings().filter_enabled and ImuSettings().noise_model == 'per_sample'
