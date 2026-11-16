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


def feedback_packet():
    from bullet_simulation_protocol.feedback import FlightFeedback, Header, TimeStamp, GroundTruth, Sensors
    header = Header('world', TimeStamp(2, 500000000))
    return FlightFeedback(header, 'drone_1', GroundTruth(header, (1., 2., 3.),
        (0., 0., 0., 1.), 3., (4., 5., 6.), (0., 0., 0.), (10., 20., 30., 40.)),
        Sensors(None, None, None, None), None).to_dict()


def test_feedback_roundtrip_and_removed_schema():
    from bullet_simulation_protocol.feedback import FlightFeedback
    from bullet_simulation_protocol.flight import validate_feedback
    packet = feedback_packet()
    assert FlightFeedback.from_dict(packet).to_dict() == packet
    assert decode(encode(packet, validate_feedback), validate_feedback) == packet
    for name in ('source', 'schema_version', 'armed', 'mode', 'last_applied_sequence',
                 'owner_id', 'state', 'reason', 'takeoff_height_m'):
        with pytest.raises(ValueError):
            validate_feedback(dict(packet, **{name: None}))
    with pytest.raises(ValueError):
        schema_bytes('flight_feedback.json')


@pytest.mark.parametrize('timestamp', [dict(sec=-1, nanosec=0), dict(sec=True, nanosec=0),
                                      dict(sec=0, nanosec=1000000000)])
def test_invalid_feedback_timestamps(timestamp):
    from bullet_simulation_protocol.feedback import FlightFeedback
    packet = feedback_packet()
    packet['header']['time_stamp'] = timestamp
    with pytest.raises(ValueError):
        FlightFeedback.from_dict(packet)


def test_feedback_sensor_frames_and_measurement_time():
    from bullet_simulation_protocol.feedback import FlightFeedback
    packet = feedback_packet()
    body = dict(frame_id='drone_1/base_link', time_stamp=dict(sec=1, nanosec=0))
    packet['sensors']['imu'] = dict(header=body, specific_force_m_s2=[0., 0., 9.81],
                                   angular_velocity_rad_s=[0., 0., 0.])
    packet['sensors']['barometer'] = dict(header=packet['header'], pressure_pa=101325., altitude_m=2.9)
    packet['sensors']['rangefinder'] = dict(header=body, distance_cm=None, valid=False)
    packet['sensors']['gps'] = dict(header=dict(frame_id='earth', time_stamp=body['time_stamp']),
        latitude_deg=32., longitude_deg=34., altitude_m=100., velocity_enu_m_s=[1., 2., 3.])
    packet['attitude_estimate'] = dict(header=packet['header'], orientation_xyzw=[0., 0., 0., 1.])
    assert FlightFeedback.from_dict(packet).to_dict() == packet
    packet['sensors']['rangefinder']['valid'] = True
    with pytest.raises(ValueError):
        FlightFeedback.from_dict(packet)


@pytest.mark.parametrize('field,value', [('altitude_m', 5.), ('position_m', [0., float('nan'), 3.]),
    ('orientation_xyzw', [0., 0., 0., 0.]), ('rotor_speeds_rad_s', [0., 0., 0., -1.])])
def test_invalid_ground_truth(field, value):
    from bullet_simulation_protocol.feedback import FlightFeedback
    packet = feedback_packet()
    packet['ground_truth'][field] = value
    with pytest.raises(ValueError):
        FlightFeedback.from_dict(packet)


def test_removed_flight_config_schema_is_unavailable():
    with pytest.raises(ValueError, match="unknown simulation schema"):
        schema_bytes("flight_config.json")
