"""ROS-shaped static pinhole calibration, independent of ROS and JSON Schema."""
from dataclasses import dataclass, field, asdict, fields
from math import isfinite, pi, tan
import json

CAMERA_INFO_TOPIC = b"camera/front/camera_info"
MAX_DIMENSION_PX = 8192
MAX_CAPTURE_HZ = 240
MAX_IMAGE_BYTES = 64 * 1024 * 1024


@dataclass(frozen=True)
class Stamp:
    sec: int = 0
    nanosec: int = 0

    def __post_init__(self):
        if type(self.sec) is not int or type(self.nanosec) is not int or self.sec != 0 or self.nanosec != 0:
            raise ValueError("static calibration requires zero stamp")


@dataclass(frozen=True)
class Header:
    stamp: Stamp = field(default_factory=Stamp)
    frame_id: str = "camera_front_optical"


@dataclass(frozen=True)
class RegionOfInterest:
    x_offset: int = 0
    y_offset: int = 0
    height: int = 0
    width: int = 0
    do_rectify: bool = False

    def __post_init__(self):
        if any(type(v) is not int or v != 0 for v in (self.x_offset, self.y_offset, self.height, self.width)) or type(self.do_rectify) is not bool or self.do_rectify:
            raise ValueError("only full-frame ROI is supported")


@dataclass(frozen=True)
class CameraInfo:
    width: int
    height: int
    k: tuple[float, ...]
    p: tuple[float, ...]
    header: Header = field(default_factory=Header)
    distortion_model: str = "plumb_bob"
    d: tuple[float, ...] = (0., 0., 0., 0., 0.)
    r: tuple[float, ...] = (1., 0., 0., 0., 1., 0., 0., 0., 1.)
    binning_x: int = 0
    binning_y: int = 0
    roi: RegionOfInterest = field(default_factory=RegionOfInterest)

    def __post_init__(self):
        for size in (self.width, self.height):
            if type(size) is not int or not 1 <= size <= MAX_DIMENSION_PX:
                raise ValueError("invalid camera dimensions")
        if self.width * self.height * 3 > MAX_IMAGE_BYTES or type(self.binning_x) is not int or type(self.binning_y) is not int:
            raise ValueError("invalid camera size or binning")
        for name, count in (("k", 9), ("p", 12), ("d", 5), ("r", 9)):
            values = getattr(self, name)
            if not isinstance(values, tuple) or len(values) != count or any(type(v) not in (int, float) or not isfinite(v) for v in values):
                raise ValueError("invalid camera matrix")
        if self.header != Header() or self.roi != RegionOfInterest() or self.distortion_model != "plumb_bob" or self.d != (0.,)*5 or self.r != (1.,0.,0.,0.,1.,0.,0.,0.,1.) or self.binning_x != 0 or self.binning_y != 0:
            raise ValueError("unsupported camera calibration")
        fx, fy, cx, cy = self.k[0], self.k[4], self.k[2], self.k[5]
        if fx <= 0 or fy != fx or cx != self.width/2 or cy != self.height/2 or self.k != (fx,0.,cx,0.,fy,cy,0.,0.,1.) or self.p != (fx,0.,cx,0.,0.,fy,cy,0.,0.,0.,1.,0.):
            raise ValueError("unsupported pinhole projection")

    @classmethod
    def from_config(cls, camera):
        width, height, fov = camera["width_px"], camera["height_px"], camera["fov_x_deg"]
        if type(fov) not in (int, float) or not isfinite(fov) or not 0 < fov < 180:
            raise ValueError("invalid horizontal FOV")
        if any(type(v) is not int or not 1 <= v <= MAX_DIMENSION_PX for v in (width, height)):
            raise ValueError("invalid camera dimensions")
        focal = width / (2 * tan(fov * pi / 360))
        cx, cy = width/2, height/2
        return cls(width, height, (focal,0.,cx,0.,focal,cy,0.,0.,1.), (focal,0.,cx,0.,0.,focal,cy,0.,0.,0.,1.,0.))


@dataclass(frozen=True)
class CameraInfoMessage:
    stream_id: str
    capture_fps_hz: float
    camera_info: CameraInfo
    schema_version: int = 1

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version != 1 or not isinstance(self.stream_id, str) or not 0 < len(self.stream_id) <= 128 or type(self.capture_fps_hz) not in (int, float) or not isfinite(self.capture_fps_hz) or not 0 < self.capture_fps_hz <= MAX_CAPTURE_HZ or not isinstance(self.camera_info, CameraInfo):
            raise ValueError("invalid camera info envelope")

    def to_dict(self):
        return json.loads(json.dumps(asdict(self), allow_nan=False))

    @classmethod
    def from_dict(cls, value):
        try:
            data = dict(value)
            if set(data) != {f.name for f in fields(cls)}:
                raise ValueError("invalid camera envelope fields")
            camera = dict(data.pop("camera_info"))
            if set(camera) != {f.name for f in fields(CameraInfo)}:
                raise ValueError("invalid camera fields")
            header = dict(camera.pop("header"))
            camera["header"] = Header(stamp=Stamp(**header.pop("stamp")), **header)
            camera["roi"] = RegionOfInterest(**camera["roi"])
            for key in ("d", "k", "r", "p"):
                camera[key] = tuple(camera[key])
            return cls(camera_info=CameraInfo(**camera), **data)
        except (TypeError, KeyError) as error:
            raise ValueError("invalid camera info fields") from error
