from dataclasses import replace
from math import pi, sqrt
from statistics import mean, stdev
from unittest.mock import Mock

import pybullet as p
import pytest
import yaml

from bullet_drone.commands import RotorCommand
from bullet_drone.config import (ForcesSettings, WindSettings, ConfigurationError, YamlConfigLoader,
                                 BodyDragSettings, InflowSettings, GroundEffectSettings, PropwashSettings)
from bullet_drone.contracts import BackendError, LifecycleError
from bullet_drone.forces import body_drag, rotor_drag, angular_damping, inflow, flapping, gyroscopic_torque, ground_effect, propwash
from bullet_drone.forces.chain import ForceChain
from bullet_drone.forces.wind import WindModel
from bullet_drone.geometry import body_to_world, from_rpy
from bullet_drone.models import DroneState, ForceOutput, PropellerOutput, SurfaceHit
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.propellers import IdealPropellerModel
from bullet_drone.vehicle import load_racer_parameters


ZERO = (0.0, 0.0, 0.0)
UP = (0.0, 0.0, 1.0)
AIR = (1.0, -2.0, 3.0)
SPEEDS = (100.0, 200.0, 300.0, 400.0)
UNIT_COEFFICIENTS = (1.0, 1.0, 1.0)
EFFECT_NAMES = ('body_drag', 'rotor_drag', 'angular_damping', 'inflow', 'flapping', 'gyroscopic', 'ground_effect', 'propwash')
DENSITY_KG_M3 = 1.225
ROTOR_RADIUS_M = 0.0635
WIND_SAMPLES = 6000
WIND_SAMPLE_DT_S = 0.05
GUST_SIGMA_M_S = 2.0
GUST_CORRELATION_TIME_S = 0.1


def source(**kwargs):
    values = dict(episode_id='episode', step_index=0, simulation_time_s=0.0, position_m=(0,0,1),
                  orientation_xyzw=(0,0,0,1), linear_velocity_m_s=AIR, angular_velocity_rad_s=(.1,.2,.3))
    return DroneState(**(values | kwargs))


def one_effect(name):
    settings = ForcesSettings.ideal()
    return replace(settings, **{name: replace(getattr(settings, name), enabled=True)})


def test_body_drag_direction_and_quadratic_scaling():
    force = body_drag.calculate(AIR, DENSITY_KG_M3, UNIT_COEFFICIENTS)
    assert force == pytest.approx(tuple(-.5 * DENSITY_KG_M3 * sqrt(14) * v for v in AIR))
    assert sum(f*v for f,v in zip(force,AIR)) < 0
    assert body_drag.calculate(tuple(2*v for v in AIR), DENSITY_KG_M3, UNIT_COEFFICIENTS) == pytest.approx(tuple(4*f for f in force))
    assert body_drag.calculate(ZERO,DENSITY_KG_M3,UNIT_COEFFICIENTS) == ZERO


def test_rotor_drag_scaling_and_zero_conditions():
    force = rotor_drag.calculate(AIR,SPEEDS,UNIT_COEFFICIENTS)
    assert force == tuple(-sum(SPEEDS)*v for v in AIR)
    assert rotor_drag.calculate(tuple(2*v for v in AIR),SPEEDS,UNIT_COEFFICIENTS) == tuple(2*f for f in force)
    assert rotor_drag.calculate(AIR,tuple(2*v for v in SPEEDS),UNIT_COEFFICIENTS) == tuple(2*f for f in force)
    assert rotor_drag.calculate(AIR,(0,)*4,UNIT_COEFFICIENTS) == ZERO
    assert rotor_drag.calculate(ZERO,SPEEDS,UNIT_COEFFICIENTS) == ZERO


def test_damping_dissipates_rotational_energy():
    torque = angular_damping.calculate(AIR,UNIT_COEFFICIENTS)
    assert torque == tuple(-v for v in AIR)
    assert sum(t*v for t,v in zip(torque,AIR)) < 0
    assert angular_damping.calculate(ZERO,UNIT_COEFFICIENTS) == ZERO


def test_inflow_and_flapping_bounds_and_per_rotor_scaling():
    assert inflow.multiplier(ZERO,100,ROTOR_RADIUS_M,1,.5) == 1
    assert inflow.multiplier(AIR,0,ROTOR_RADIUS_M,1,.5) == 1
    assert inflow.multiplier((1000,0,0),100,ROTOR_RADIUS_M,1,.5) == .5
    assert inflow.multiplier((1,0,0),100,ROTOR_RADIUS_M,1,.5) == pytest.approx(1-1/(100*ROTOR_RADIUS_M))
    force = flapping.calculate(AIR,2,100,ROTOR_RADIUS_M,.1)
    assert force == pytest.approx((-.2/(100*ROTOR_RADIUS_M),.4/(100*ROTOR_RADIUS_M),0))
    assert sum(f*v for f,v in zip(force,AIR)) < 0
    assert flapping.calculate(AIR,2,0,ROTOR_RADIUS_M,.1) == ZERO
    assert flapping.calculate(AIR,0,100,ROTOR_RADIUS_M,.1) == ZERO
    assert flapping.calculate(AIR,4,100,ROTOR_RADIUS_M,.1) == pytest.approx(tuple(2*f for f in force))


def test_gyro_body_reaction_signs_and_cancellation():
    assert gyroscopic_torque.calculate(AIR,(100,)*4,(1,-1,1,-1),.01) == ZERO
    assert gyroscopic_torque.calculate((0,2,0),(100,0,0,0),(1,-1,1,-1),.01) == (-2,0,0)
    assert gyroscopic_torque.calculate((2,0,0),(100,0,0,0),(1,-1,1,-1),.01) == (0,2,0)
    assert gyroscopic_torque.calculate((0,2,0),(100,0,0,0),(-1,1,-1,1),.01) == (2,0,0)


def test_ground_gain_surface_alignment_range_and_bounds():
    settings = GroundEffectSettings()
    assert ground_effect.multiplier(None,UP,ROTOR_RADIUS_M,settings) == 1
    assert ground_effect.multiplier(SurfaceHit(2,UP),UP,ROTOR_RADIUS_M,settings) == 1
    assert ground_effect.multiplier(SurfaceHit(.001,UP),UP,ROTOR_RADIUS_M,settings) == settings.maximum_multiplier
    assert ground_effect.multiplier(SurfaceHit(.1,UP),(0,0,-1),ROTOR_RADIUS_M,settings) == 1
    upright = ground_effect.multiplier(SurfaceHit(.1,UP),UP,ROTOR_RADIUS_M,settings)
    tilted = ground_effect.multiplier(SurfaceHit(.1,UP),(sqrt(.5),0,sqrt(.5)),ROTOR_RADIUS_M,settings)
    assert tilted - 1 == pytest.approx((upright-1)*.5)


def wake(point, *, positions=(ZERO,), thrusts=(2.0,), settings=PropwashSettings(), exclude=None):
    return propwash.velocity_at(point,positions,UP,thrusts,DENSITY_KG_M3,ROTOR_RADIUS_M,settings,exclude_rotor=exclude)


def test_wake_direction_decay_superposition_and_exclusion():
    near = wake((0,0,-.1))
    assert near[:2] == (0,0) and near[2] < 0
    assert abs(wake((0,0,-1))[2]) < abs(near[2])
    assert abs(wake((.1,0,-.1))[2]) < abs(near[2])
    assert wake((0,0,.1)) == wake(ZERO) == ZERO
    assert wake((0,0,-.1),exclude=0) == ZERO
    assert wake((0,0,-.1),thrusts=(0,)) == ZERO
    duplicate = wake((0,0,-.1),positions=(ZERO,ZERO),thrusts=(.1,.1))
    assert duplicate == pytest.approx(tuple(2*v for v in wake((0,0,-.1),thrusts=(.1,))))
    assert abs(wake((0,0,-.1),settings=PropwashSettings(maximum_speed_m_s=1))[2]) == pytest.approx(1)


def test_constant_wind_disable_and_gust_reproducibility():
    assert WindModel(WindSettings(velocity_world_m_s=AIR)).velocity(10) == AIR
    assert WindModel(WindSettings(enabled=False,velocity_world_m_s=AIR)).velocity(10) == ZERO
    model = WindModel(WindSettings(behavior='gusts',gust_stddev_m_s=(1,2,3)))
    assert model.velocity(0) == ZERO
    first = model.velocity(.1)
    assert first != ZERO and model.velocity(.1) == first
    model.reset()
    assert model.velocity(.1) == first
    with pytest.raises(ValueError): model.velocity(0)


@pytest.mark.parametrize('dt', [WIND_SAMPLE_DT_S, WIND_SAMPLE_DT_S/2])
def test_gust_stationary_statistics_across_timesteps(dt):
    model = WindModel(WindSettings(behavior='gusts',gust_stddev_m_s=(GUST_SIGMA_M_S,0,0),
                                   correlation_time_s=GUST_CORRELATION_TIME_S))
    samples = [model.velocity(i*dt)[0] for i in range(1,WIND_SAMPLES+1)]
    assert mean(samples) == pytest.approx(0,abs=.2)
    assert stdev(samples) == pytest.approx(GUST_SIGMA_M_S,abs=.2)


@pytest.mark.parametrize('name', EFFECT_NAMES)
def test_each_effect_switch_and_ideal_baseline(name):
    vehicle = load_racer_parameters()
    baseline = IdealPropellerModel(vehicle).calculate(SPEEDS)
    scene = Mock(surface_below=Mock(return_value=SurfaceHit(.02,UP)))
    state = source(position_m=(0,0,.05),angular_velocity_rad_s=(.5,.2,.3))
    ideal = ForceChain(ForcesSettings.ideal(),WindSettings(),vehicle).calculate(state,SPEEDS,baseline,scene)
    settings = one_effect(name)
    active = ForceChain(settings,WindSettings(),vehicle).calculate(state,SPEEDS,baseline,scene)
    # Propwash changes airflow, so needs a receiving drag model to produce a force.
    if name == 'propwash':
        settings = replace(settings,body_drag=BodyDragSettings())
        active = ForceChain(settings,WindSettings(),vehicle).calculate(state,SPEEDS,baseline,scene)
        no_wash = ForceChain(replace(settings,propwash=PropwashSettings(enabled=False)),WindSettings(),vehicle).calculate(state,SPEEDS,baseline,scene)
        assert active != no_wash
    else:
        assert active != ideal
    assert ideal == ForceOutput(baseline.forces_n,ZERO,baseline.reaction_torque_nm)
    if name != 'ground_effect': scene.surface_below.assert_not_called()


def test_matching_wind_cancels_drag_and_yaw_rotates_equivalent_world_force():
    vehicle = load_racer_parameters()
    baseline = IdealPropellerModel(vehicle).calculate((0,)*4)
    settings = one_effect('body_drag')
    no_hit = Mock()
    aligned = source(linear_velocity_m_s=AIR,angular_velocity_rad_s=ZERO)
    cancelled = ForceChain(settings,WindSettings(velocity_world_m_s=AIR),vehicle).calculate(aligned,(0,)*4,baseline,no_hit)
    assert cancelled.body_force_n == ZERO
    # Use isotropic drag for yaw equivalence.
    settings = replace(settings,body_drag=BodyDragSettings(cd_area_m2=(.02,)*3))
    turn = from_rpy((0,0,pi/2))
    rotated = ForceChain(settings,WindSettings(),vehicle).calculate(replace(aligned,orientation_xyzw=turn),(0,)*4,baseline,no_hit)
    assert body_to_world(rotated.body_force_n,turn) == pytest.approx(body_drag.calculate(AIR,DENSITY_KG_M3,(.02,)*3))


def test_surface_query_skips_drone_and_finds_raised_surface(config):
    physics = PyBulletPhysics(config)
    try:
        physics.reset()
        assert physics.surface_below((0,0,2),3).distance_m == pytest.approx(2)
        collision = p.createCollisionShape(p.GEOM_BOX,halfExtents=(.2,.2,.1),physicsClientId=physics._client)
        p.createMultiBody(baseMass=0,baseCollisionShapeIndex=collision,basePosition=(.4,0,.4),physicsClientId=physics._client)
        assert physics.surface_below((.4,0,1),2).distance_m == pytest.approx(.5)
        assert physics.surface_below((0,0,2),.1) is None
    finally:
        physics.close()


def test_default_physics_adds_drag_and_resets_gusts(config):
    config = replace(config,forces=one_effect('body_drag'),wind=WindSettings(behavior='gusts',velocity_world_m_s=(1,0,0),gust_stddev_m_s=(.1,)*3))
    physics = PyBulletPhysics(config)
    try:
        initial = physics.reset()
        first = physics.step()
        assert first.linear_velocity_m_s[0] > 0
        sequence = [physics.step().linear_velocity_m_s for _ in range(5)]
        physics.reset()
        assert physics.step().linear_velocity_m_s == first.linear_velocity_m_s
        assert [physics.step().linear_velocity_m_s for _ in range(5)] == sequence
    finally:
        physics.close()


def test_invalid_chain_output_applies_nothing_and_requires_reset(config,monkeypatch):
    physics = PyBulletPhysics(config)
    try:
        physics.reset()
        monkeypatch.setattr(physics._forces,'calculate',Mock(return_value=ForceOutput(((0,0,1),)*4,(float('nan'),0,0),ZERO)))
        apply_force = Mock()
        monkeypatch.setattr(p,'applyExternalForce',apply_force)
        with pytest.raises(BackendError): physics.step()
        apply_force.assert_not_called()
        with pytest.raises(LifecycleError): physics.step()
    finally:
        physics.close()


@pytest.mark.parametrize('kwargs', [dict(air_density_kg_m3=0),dict(rotor_radius_m=float('nan')),dict(body_drag={})])
def test_force_settings_reject_invalid_values(kwargs):
    with pytest.raises(ConfigurationError): ForcesSettings(**kwargs)


@pytest.mark.parametrize('settings_type,kwargs', [
    (BodyDragSettings,dict(enabled='yes')),(BodyDragSettings,dict(cd_area_m2=(-1,0,0))),
    (InflowSettings,dict(minimum_multiplier=2)),(GroundEffectSettings,dict(maximum_multiplier=.5)),
    (GroundEffectSettings,dict(minimum_distance_m=2)),(PropwashSettings,dict(maximum_speed_m_s=0)),
    (WindSettings,dict(seed=True)),(WindSettings,dict(behavior='random')),
    (WindSettings,dict(gust_stddev_m_s=(0,-1,0))),(WindSettings,dict(correlation_time_s=0))])
def test_effect_parameters_reject_invalid_values(settings_type,kwargs):
    with pytest.raises(ConfigurationError): settings_type(**kwargs)


def test_yaml_defaults_roundtrip_flags_and_unknown_fields(tmp_path):
    data = yaml.safe_load(open('configs/basic.yaml'))
    data.pop('forces',None)
    data.pop('wind',None)
    path = tmp_path/'forces.yaml'
    path.write_text(yaml.safe_dump(data))
    loaded = YamlConfigLoader().load(path)
    assert loaded.forces == ForcesSettings() and loaded.wind == WindSettings()
    data['forces'] = {name:{'enabled':False} for name in EFFECT_NAMES}
    data['wind'] = dict(behavior='gusts',velocity_world_m_s=list(AIR),gust_stddev_m_s=[1,2,3])
    path.write_text(yaml.safe_dump(data))
    loaded = YamlConfigLoader().load(path)
    assert loaded.forces == ForcesSettings.ideal()
    assert loaded.wind.velocity_world_m_s == AIR
    for section, value in [('forces',{'unknown':1}),('forces',{'body_drag':{'cd_area_m2':[0,0,-1]}}),
                           ('forces',{'inflow':{'enabled':0}}),('wind',{'behavior':'unknown'}),('wind',{'unknown':1})]:
        original = data[section]
        data[section] = value
        path.write_text(yaml.safe_dump(data))
        with pytest.raises(ConfigurationError): YamlConfigLoader().load(path)
        data[section] = original


def test_rotor_point_airflow_creates_offset_flapping_torque_once(config,monkeypatch):
    physics = PyBulletPhysics(replace(config,forces=one_effect('flapping')))
    try:
        initial = physics.reset()
        p.resetBaseVelocity(physics._body,angularVelocity=(0,0,1),physicsClientId=physics._client)
        apply_force = Mock(wraps=p.applyExternalForce)
        apply_torque = Mock(wraps=p.applyExternalTorque)
        monkeypatch.setattr(p,'applyExternalForce',apply_force)
        monkeypatch.setattr(p,'applyExternalTorque',apply_torque)
        result = physics.step(RotorCommand((500,)*4,initial.episode_id,1))
        assert result.angular_velocity_rad_s[2] < 1
        assert apply_force.call_count == 5  # Four rotor offsets and one center-of-mass force.
        assert apply_torque.call_count == 1
        assert apply_torque.call_args.args[2] == ZERO  # Offset flapping supplies the yaw resistance.
        assert [call.args[3] for call in apply_force.call_args_list] == [*physics._vehicle.rotor_positions_m,ZERO]
    finally:
        physics.close()


def test_angular_damping_reduces_rate_in_real_physics(config):
    physics = PyBulletPhysics(replace(config,forces=one_effect('angular_damping'),
                                     simulation=replace(config.simulation,gravity_m_s2=ZERO)))
    try:
        physics.reset()
        p.resetBaseVelocity(physics._body,angularVelocity=(0,0,2),physicsClientId=physics._client)
        for _ in range(120): state = physics.step()
        assert 0 < state.angular_velocity_rad_s[2] < 2
    finally:
        physics.close()


def test_surface_query_failure_invalidates_physics(config,monkeypatch):
    physics = PyBulletPhysics(replace(config,forces=ForcesSettings()))
    try:
        physics.reset()
        monkeypatch.setattr(physics,'surface_below',Mock(side_effect=ValueError('scene query failure')))
        with pytest.raises(BackendError,match='scene query failure'): physics.step()
        with pytest.raises(LifecycleError): physics.step()
    finally:
        physics.close()


def test_stopped_rotors_do_not_generate_wake_from_custom_baseline():
    vehicle = load_racer_parameters()
    settings = replace(one_effect('propwash'), body_drag=BodyDragSettings())
    chain = ForceChain(settings,WindSettings(),vehicle)
    custom = PropellerOutput(((0,0,2),)*4,ZERO)
    result = chain.calculate(source(linear_velocity_m_s=ZERO,angular_velocity_rad_s=ZERO),
                             (0,)*4,custom,Mock())
    assert result.body_force_n == ZERO
    assert result.rotor_forces_n == custom.forces_n
