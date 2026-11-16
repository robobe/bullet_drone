import json

import pytest
from bullet_simulation_protocol import schema_bytes
from bullet_simulation_protocol.flight import validate_source, decode, encode
from bullet_image_transport.shm import FrameWriter, FrameReader
from jsonschema import Draft202012Validator


SOURCE = dict(physics_id='physics', episode_id='episode', stream_id='stream',
              step_index=7, simulation_time_s=.1, measurement_us=1234567)


def test_source_and_json_validation():
    assert decode(encode(SOURCE, validate_source), validate_source) == SOURCE
    with pytest.raises(ValueError):
        validate_source(dict(SOURCE, simulation_time_s=10**1000))
    with pytest.raises(ValueError):
        decode(b'{"value":NaN}', validate_source)
    with pytest.raises(ValueError):
        decode(b'[' * 1200 + b']' * 1200, validate_source)


def test_stamped_descriptor_preserves_source_and_binary_header():
    writer = FrameWriter(2, 2)
    reader = FrameReader('camera/front/image')
    try:
        message = writer.write('camera/front/image', b'\x01' * 12, SOURCE['measurement_us'] * 1000, source=SOURCE)
        metadata, pixels = reader.read(message)
        assert metadata['version'] == 2 and metadata['source'] == SOURCE
        assert pixels == b'\x01' * 12
        Draft202012Validator(json.loads(__import__('bullet_image_transport').schema_bytes('image.json'))).validate(metadata)
        with pytest.raises(ValueError):
            writer.write('camera/front/image', b'\x01' * 12, 1, source=SOURCE)
    finally:
        reader.close()
        writer.close()


def test_feedback_v2_carries_selected_takeoff_and_v1_stays_valid():
    from bullet_simulation_protocol.flight import validate_feedback
    base = dict(schema_version=1, source=SOURCE, barometer_altitude_m=30,
                orientation_xyzw=[0, 0, 0, 1], armed=False, mode='angle',
                last_applied_sequence=0, owner_id=None, state='stopped', reason='spawn_applied')
    schema = Draft202012Validator(json.loads(schema_bytes('flight_feedback.json')))
    for packet in (base, dict(base, schema_version=2, takeoff_height_m=6.5)):
        validate_feedback(packet)
        schema.validate(packet)
    for value in (0, -.1, 100.1, True, float('nan'), float('inf'), None):
        with pytest.raises(ValueError):
            validate_feedback(dict(base, schema_version=2, takeoff_height_m=value))
    with pytest.raises(ValueError):
        validate_feedback(dict(base, schema_version=2))
    with pytest.raises(ValueError):
        validate_feedback(dict(base, takeoff_height_m=6.5))


def test_feedback_rangefinder_is_optional_but_strict():
    from bullet_simulation_protocol.flight import validate_feedback
    base = dict(schema_version=2, source=SOURCE, takeoff_height_m=6.5,
                barometer_altitude_m=3., orientation_xyzw=[0., 0., 0., 1.],
                armed=False, mode='angle', last_applied_sequence=0, owner_id=None,
                state='running', reason='test')
    packet = dict(base, rangefinder=dict(distance_cm=275., valid=True))
    validate_feedback(packet)
    Draft202012Validator(json.loads(schema_bytes('flight_feedback.json'))).validate(packet)
    validate_feedback(base)
    with pytest.raises(ValueError):
        validate_feedback(dict(packet, rangefinder=dict(distance_cm=1., valid=True, extra=1)))


@pytest.mark.parametrize('model', ['ideal', 'imu_complementary'])
def test_attitude_models_match_wire_schema(model):
    from bullet_simulation_protocol.flight import validate_config
    packet = dict(schema_version=1, physics_id='physics', stream_id='stream', mass_kg=1.,
                  gravity_m_s2=9.81, max_thrust_n=20., rc_angle_scale_rad=.5,
                  rc_yaw_rate_scale_rad_s=2., physics_hz=240, feedback_hz=120, command_hz=100,
                  aetr_channels=[1, 2, 3, 4], arm_channel=5, mode_channel=6,
                  camera=dict(width_px=640, height_px=480, fov_x_deg=90.), attitude_model=model)
    schema = Draft202012Validator(json.loads(schema_bytes('flight_config.json')))
    validate_config(packet)
    schema.validate(packet)
    invalid = dict(packet, attitude_model='unknown')
    with pytest.raises(ValueError):
        validate_config(invalid)
    assert not schema.is_valid(invalid)
