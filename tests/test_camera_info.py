import math
import pytest
from bullet_simulation_protocol.camera_info import CameraInfo, CameraInfoMessage


def test_pinhole_round_trip_and_optical_conventions():
    info = CameraInfo.from_config(dict(width_px=640, height_px=480, fov_x_deg=90))
    assert info.k[0] == pytest.approx(320)
    assert info.k[4] == info.k[0]
    assert info.k[2:6:3] == (320, 240)
    message = CameraInfoMessage('stream', 30, info)
    wire = message.to_dict()
    assert isinstance(wire['camera_info']['k'], list)
    assert CameraInfoMessage.from_dict(wire) == message
    wire['camera_info']['d'][0] = .1
    with pytest.raises(ValueError):
        CameraInfoMessage.from_dict(wire)


@pytest.mark.parametrize('fov', [0,180,float('nan'),True])
def test_invalid_camera(fov):
    with pytest.raises(ValueError):
        CameraInfo.from_config(dict(width_px=640,height_px=480,fov_x_deg=fov))


def test_dictionary_rejects_missing_extra_and_boolean_integer_fields():
    message = CameraInfoMessage('stream', 30, CameraInfo.from_config(
        dict(width_px=640, height_px=480, fov_x_deg=90)))
    wire = message.to_dict()
    wire['camera_info']['binning_x'] = False
    with pytest.raises(ValueError):
        CameraInfoMessage.from_dict(wire)
    wire = message.to_dict()
    del wire['camera_info']['header']
    with pytest.raises(ValueError):
        CameraInfoMessage.from_dict(wire)
    wire = message.to_dict()
    wire['extra'] = 1
    with pytest.raises(ValueError):
        CameraInfoMessage.from_dict(wire)
