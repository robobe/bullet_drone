import pytest
from bullet_simulation_protocol.pose import PoseMessage, PoseHeader
from bullet_simulation_protocol.feedback import TimeStamp


def test_pose_roundtrip():
    pose = PoseMessage(PoseHeader(240, TimeStamp(1, 0)), 'iris', (0., 0., 3.), (0., 0., 0., 1.))
    assert PoseMessage.from_dict(pose.to_dict()) == pose
    assert set(pose.to_dict()) == {'header', 'drone_id', 'position_m', 'orientation_xyzw'}


@pytest.mark.parametrize('field,value', [('position_m', [0., float('nan'), 0.]),
    ('orientation_xyzw', [0., 0., 0., 0.]), ('schema_version', 2), ('drone_id', '')])
def test_pose_rejects_invalid_fields(field, value):
    packet = PoseMessage(PoseHeader(0, TimeStamp(0, 0)), 'drone', (0., 0., 0.), (0., 0., 0., 1.)).to_dict()
    packet[field] = value
    with pytest.raises(ValueError):
        PoseMessage.from_dict(packet)


@pytest.mark.parametrize('header', [
    {'fid': True, 'time_stamp': {'sec': 0, 'nanosec': 0}},
    {'fid': 1.5, 'time_stamp': {'sec': 0, 'nanosec': 0}},
    {'fid': 1, 'time_stamp': {'sec': 0.0, 'nanosec': 0}},
    {'fid': 1, 'time_stamp': {'sec': True, 'nanosec': 0}},
    {'fid': 1, 'time_stamp': {'sec': 0, 'nanosec': 1.5}},
    {'fid': 1, 'time_stamp': {'sec': -1, 'nanosec': 0}},
    {'fid': 1, 'time_stamp': {'sec': 0, 'nanosec': 1000000000}},
])
def test_pose_header_rejects_noninteger_or_out_of_range_values(header):
    with pytest.raises(ValueError):
        PoseHeader.from_dict(header)
