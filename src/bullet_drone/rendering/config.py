"""Validated render profiles, independent of physics and networking."""
from dataclasses import dataclass, field
from math import isfinite
from pathlib import Path
import yaml
from bullet_drone.config import ConfigurationError


class UnsupportedRenderSourceError(ConfigurationError):
    """Legacy standalone render profiles support ground truth only."""


def number(value, name, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
        raise ConfigurationError(f"{name} must be finite numeric data")
    if (minimum is not None and value <= minimum) or (maximum is not None and value >= maximum):
        raise ConfigurationError(f"{name} is outside its supported range")
    return float(value)


def vector(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ConfigurationError(f"{name} requires {size} values")
    return tuple(number(v, name) for v in value)


def identity(value, name):
    if not isinstance(value, str) or not value.strip() or len(value) > 128:
        raise ConfigurationError(f"{name} must be a nonempty string of at most 128 characters")
    return value


@dataclass(frozen=True, slots=True)
class RenderSources:
    position: str = 'ground_truth'
    altitude: str = 'ground_truth'
    attitude: str = 'ground_truth'

    def __post_init__(self):
        for name in ('position', 'altitude', 'attitude'):
            source = getattr(self, name)
            if source not in ('ground_truth', 'sensor'):
                raise ConfigurationError(f"sources.{name} must be ground_truth or sensor")
            if source == 'sensor':
                raise UnsupportedRenderSourceError(f"sources.{name}: sensor rendering is not implemented")


@dataclass(frozen=True, slots=True)
class CameraSettings:
    width_px: int = 640
    height_px: int = 480
    fov_y_deg: float = 75
    mount_position_body_m: tuple = (0.15, 0., 0.)
    downward_tilt_deg: float = 10

    def __post_init__(self):
        for name in ('width_px', 'height_px'):
            if type(getattr(self, name)) is not int or not 0 < getattr(self, name) <= 8192:
                raise ConfigurationError(f"camera.{name} must be an integer in 1..8192")
        number(self.fov_y_deg, 'camera.fov_y_deg', 0, 180)
        number(self.downward_tilt_deg, 'camera.downward_tilt_deg', -90, 90)
        object.__setattr__(self, 'mount_position_body_m', vector(self.mount_position_body_m, 3, 'camera.mount_position_body_m'))


@dataclass(frozen=True, slots=True)
class UdpSettings:
    host: str = '127.0.0.1'
    port: int = 9870

    def __post_init__(self):
        identity(self.host, 'udp.host')
        if type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ConfigurationError('udp.port must be an integer in 1..65535')


@dataclass(frozen=True, slots=True)
class RenderSettings:
    drone_id: str = 'drone_1'
    sources: RenderSources = field(default_factory=RenderSources)
    udp: UdpSettings = field(default_factory=UdpSettings)
    rate_hz: float = 60
    camera: CameraSettings = field(default_factory=CameraSettings)

    def __post_init__(self):
        identity(self.drone_id, 'drone_id')
        number(self.rate_hz, 'rate_hz', 0)
        for name, cls in (('sources', RenderSources), ('udp', UdpSettings), ('camera', CameraSettings)):
            if not isinstance(getattr(self, name), cls):
                raise ConfigurationError(f'{name} must be {cls.__name__}')


def load_render_settings(path: Path | None = None) -> RenderSettings:
    if path is None:
        return RenderSettings()
    try:
        data = yaml.safe_load(path.read_text())
        if not isinstance(data, dict):
            raise ConfigurationError('render profile must be a mapping')
        data = dict(data)
        for key, cls in (('sources', RenderSources), ('udp', UdpSettings), ('camera', CameraSettings)):
            if key in data:
                data[key] = cls(**data[key])
        return RenderSettings(**data)
    except (OSError, yaml.YAMLError, TypeError) as exc:
        raise ConfigurationError(f'Invalid render profile: {exc}') from exc
