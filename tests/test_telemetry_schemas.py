from copy import deepcopy
from dataclasses import asdict, fields, replace
import json

import pytest
import yaml
from jsonschema import Draft202012Validator, ValidationError

from bullet_drone.config import ConfigurationError, TelemetrySettings, YamlConfigLoader
from bullet_drone.environment import DroneEnvironment
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.telemetry.header import HEADER_SCHEMA, SCHEMA_VERSION
from bullet_drone.telemetry.topics import MANDATORY_TOPICS, OPTIONAL_TOPICS, TOPIC_SCHEMAS, selected_schemas
from bullet_drone.telemetry.topics import ground_truth
from bullet_drone.vehicle import load_racer_parameters


EXPECTED_TOPICS = {
    'ground_truth', 'config', 'events', 'sensors', 'rc_input', 'control',
    'allocation', 'forces', 'airflow', 'timing',
}
ZERO_VECTOR = [0.0, 0.0, 0.0]
ZERO_ROTORS = [0.0, 0.0, 0.0, 0.0]
ROTOR_ZERO_VECTORS = [ZERO_VECTOR] * 4
HEADER = {'schema_version': SCHEMA_VERSION, 'drone_id': 'drone-a', 'episode_id': 'episode-a',
          'step_index': 1, 'simulation_time_s': 0.01}
SOURCE = {'step_index': 0, 'simulation_time_s': 0.0}
WRENCH = {'thrust_n': 0.0, 'torque_nm': ZERO_VECTOR}
FORCE_OUTPUT = {'rotor_forces_n': ROTOR_ZERO_VECTORS, 'body_force_n': ZERO_VECTOR,
                'body_torque_nm': ZERO_VECTOR}
PAYLOADS = {
    'ground_truth': {'position_m': ZERO_VECTOR, 'orientation_xyzw': [0, 0, 0, 1],
                     'linear_velocity_m_s': ZERO_VECTOR, 'angular_velocity_rad_s': ZERO_VECTOR,
                     'rotor_speeds_rad_s': ZERO_ROTORS},
    'config': {'simulation_config': {}, 'vehicle_parameters': {}, 'stabilization_settings': {},
               'seeds': {'wind': 2}, 'git_commit': None},
    'events': {'event': 'reset', 'details': '', 'dropped_frames': 0, 'complete': True},
    'sensors': {'imu': None, 'barometer': None, 'rangefinder': None},
    'rc_input': {'channels': [1500] * 18},
    'control': {'mode': 'angle', 'armed': True, 'rpy': ZERO_VECTOR, 'yaw_rate_rad_s': 0.0,
                'thrust_n': 9.81, 'source': SOURCE},
    'allocation': {'allocation': {'requested': WRENCH, 'achieved': WRENCH,
                   'speeds_rad_s': ZERO_ROTORS, 'rotor_thrusts_n': ZERO_ROTORS,
                   'saturated': False, 'torque_scale': 1.0}},
    'forces': {'total': FORCE_OUTPUT, 'contributions': {'baseline': FORCE_OUTPUT},
               'inflow_multipliers': [1.0] * 4, 'ground_effect_multipliers': [1.0] * 4,
               'source': SOURCE},
    'airflow': {'ambient_wind_world_m_s': ZERO_VECTOR, 'gust_world_m_s': ZERO_VECTOR,
                'body_wake_velocity_body_m_s': ZERO_VECTOR,
                'rotor_wake_velocities_body_m_s': ROTOR_ZERO_VECTORS,
                'body_relative_velocity_body_m_s': ZERO_VECTOR,
                'rotor_relative_velocities_body_m_s': ROTOR_ZERO_VECTORS, 'source': SOURCE},
    'timing': {'simulation_elapsed_s': 0.01, 'wall_elapsed_s': 0.02, 'real_time_factor': 0.5},
}


def sample(name):
    return deepcopy({'header': HEADER, 'data': PAYLOADS[name]})


def validate(name, payload):
    Draft202012Validator(TOPIC_SCHEMAS[f'simulation/{name}']).validate(payload)


def test_registry_and_self_contained_common_header():
    assert set(TOPIC_SCHEMAS) == {f'simulation/{name}' for name in EXPECTED_TOPICS}
    for topic, schema in TOPIC_SCHEMAS.items():
        Draft202012Validator.check_schema(schema)
        assert schema['title'] == topic
        assert schema['properties']['header'] == HEADER_SCHEMA
        assert schema['properties']['header'] is not HEADER_SCHEMA
        assert schema['required'] == ['header', 'data']
        # Each schema is independently serializable and needs no external reference resolver.
        assert '$ref' not in json.dumps(schema)


@pytest.mark.parametrize('name', sorted(EXPECTED_TOPICS))
def test_representative_payloads(name):
    validate(name, json.loads(json.dumps(sample(name), allow_nan=False)))


@pytest.mark.parametrize('name', sorted(EXPECTED_TOPICS))
@pytest.mark.parametrize('missing', list(HEADER))
def test_header_fields_required_for_every_topic(name, missing):
    payload = sample(name)
    del payload['header'][missing]
    with pytest.raises(ValidationError):
        validate(name, payload)


@pytest.mark.parametrize('field,value', [
    ('schema_version', 2), ('schema_version', True), ('drone_id', ''), ('drone_id', '  '),
    ('drone_id', 'x' * 129), ('episode_id', ''), ('step_index', -1),
    ('step_index', True), ('simulation_time_s', -0.1), ('simulation_time_s', 'now'),
])
def test_invalid_header(field, value):
    payload = sample('ground_truth')
    payload['header'][field] = value
    with pytest.raises(ValidationError):
        validate('ground_truth', payload)


@pytest.mark.parametrize('name,field,value', [
    ('rc_input', 'channels', [1500] * 17), ('rc_input', 'channels', [1500] * 19),
    ('rc_input', 'channels', [999] * 18), ('rc_input', 'channels', [2001] * 18),
    ('rc_input', 'channels', [True] * 18), ('control', 'mode', 'joystick'),
    ('control', 'thrust_n', -1), ('ground_truth', 'position_m', [0, 0]),
    ('ground_truth', 'rotor_speeds_rad_s', [-1] * 4), ('sensors', 'rangefinder', {}),
    ('events', 'event', 'mission_takeoff'), ('timing', 'real_time_factor', -1),
])
def test_invalid_topic_data(name, field, value):
    payload = sample(name)
    payload['data'][field] = value
    with pytest.raises(ValidationError):
        validate(name, payload)


@pytest.mark.parametrize('name', ['control', 'forces', 'airflow'])
def test_source_identity_required(name):
    payload = sample(name)
    del payload['data']['source']
    with pytest.raises(ValidationError):
        validate(name, payload)
    payload = sample(name)
    payload['data']['source']['simulation_time_s'] = -1
    with pytest.raises(ValidationError):
        validate(name, payload)


@pytest.mark.parametrize('location', ['root', 'header', 'data'])
def test_unexpected_fields_rejected(location):
    payload = sample('ground_truth')
    target = payload if location == 'root' else payload[location]
    target['unexpected'] = 0
    with pytest.raises(ValidationError):
        validate('ground_truth', payload)


def test_nullable_and_acro_variants():
    payload = sample('allocation')
    payload['data']['allocation'] = None
    validate('allocation', payload)
    payload = sample('control')
    payload['data'].update(mode='acro', yaw_rate_rad_s=None)
    validate('control', payload)
    payload = sample('timing')
    payload['data'].update(wall_elapsed_s=0, real_time_factor=None)
    validate('timing', payload)


def test_live_ground_truth_and_config_payloads(config):
    for drone_id in ('drone-a', 'drone-b'):
        cfg = replace(config, drone=replace(config.drone, drone_id=drone_id))
        env = DroneEnvironment(PyBulletPhysics(cfg))
        try:
            initial = env.reset()
            stepped = env.step()
            for observation in (initial, stepped):
                data = asdict(observation.ground_truth)
                header = {key: data.pop(key) for key in HEADER if key != 'schema_version'}
                header['schema_version'] = SCHEMA_VERSION
                payload = json.loads(json.dumps({'header': header, 'data': data}, allow_nan=False))
                Draft202012Validator(ground_truth.SCHEMA).validate(payload)
                assert payload['header']['drone_id'] == drone_id
                assert payload['header']['step_index'] == observation.ground_truth.step_index
            snapshot = sample('config')
            snapshot['data'].update(simulation_config=asdict(cfg), vehicle_parameters=asdict(load_racer_parameters()))
            validate('config', json.loads(json.dumps(snapshot, allow_nan=False)))
        finally:
            env.close()


def test_selection_defaults_and_all_disabled():
    assert selected_schemas(TelemetrySettings()) == TOPIC_SCHEMAS
    disabled = TelemetrySettings(**{field.name: False for field in fields(TelemetrySettings)})
    assert set(selected_schemas(disabled)) == MANDATORY_TOPICS
    assert set(OPTIONAL_TOPICS) == {field.name for field in fields(TelemetrySettings)}


@pytest.mark.parametrize('flag', list(OPTIONAL_TOPICS))
def test_independent_selection(flag):
    selected = selected_schemas(TelemetrySettings(**{flag: False}))
    assert set(selected) == set(TOPIC_SCHEMAS) - {OPTIONAL_TOPICS[flag]}
    assert MANDATORY_TOPICS <= selected.keys()


@pytest.mark.parametrize('flag', list(OPTIONAL_TOPICS))
@pytest.mark.parametrize('value', [0, 1, 'false', None, []])
def test_python_config_requires_boolean(flag, value):
    with pytest.raises(ConfigurationError, match=f'telemetry.{flag}'):
        TelemetrySettings(**{flag: value})


def load_yaml(tmp_path, telemetry):
    data = {'simulation': {'timestep_s': 0.01, 'max_steps': 10, 'gravity_m_s2': [0, 0, -9.81]},
            'drone': {'mass_kg': 1, 'initial_position_m': [0, 0, 1]}, 'telemetry': telemetry}
    path = tmp_path / 'telemetry.yaml'
    path.write_text(yaml.safe_dump(data))
    return YamlConfigLoader().load(path)


def test_yaml_defaults_and_independent_selection(tmp_path, config):
    assert load_yaml(tmp_path, {}).telemetry == config.telemetry == TelemetrySettings()
    for flag in OPTIONAL_TOPICS:
        settings = load_yaml(tmp_path, {flag: False}).telemetry
        assert set(selected_schemas(settings)) == set(TOPIC_SCHEMAS) - {OPTIONAL_TOPICS[flag]}
    assert YamlConfigLoader().load('configs/basic.yaml').telemetry == TelemetrySettings()


@pytest.mark.parametrize('value', [None, [], False, {'control': 'false'}, {'control': 0},
                                    {'control': None}, {'unknown': True}, {'ground_truth': False},
                                    {'config': False}, {'events': False}])
def test_invalid_yaml_settings(tmp_path, value):
    with pytest.raises(ConfigurationError):
        load_yaml(tmp_path, value)


def test_simulation_config_requires_settings_instance(config):
    with pytest.raises(ConfigurationError, match='telemetry must be TelemetrySettings'):
        replace(config, telemetry={})
