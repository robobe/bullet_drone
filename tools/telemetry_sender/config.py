"""Strict configuration for the example-side UDP sender."""

from dataclasses import dataclass, fields
from ipaddress import IPv4Address, AddressValueError
from pathlib import Path

import yaml

from bullet_drone.config import ConfigurationError, _number


DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = 9870
DEFAULT_SEND_RATE_HZ = 5.0
DEFAULT_QUEUE_CAPACITY = 256
DEFAULT_STARTUP_TIMEOUT_S = 3.0
DEFAULT_SHUTDOWN_TIMEOUT_S = 3.0
MIN_PORT = 1
MAX_PORT = 65535


@dataclass(frozen=True, slots=True)
class SenderSettings:
    host: str = DEFAULT_HOST
    port: int = DEFAULT_PORT
    send_rate_hz: float = DEFAULT_SEND_RATE_HZ
    queue_capacity: int = DEFAULT_QUEUE_CAPACITY
    startup_timeout_s: float = DEFAULT_STARTUP_TIMEOUT_S
    shutdown_timeout_s: float = DEFAULT_SHUTDOWN_TIMEOUT_S

    def __post_init__(self) -> None:
        try:
            if not isinstance(self.host, str):
                raise AddressValueError('expected a string')
            IPv4Address(self.host)
        except AddressValueError as exc:
            raise ConfigurationError('sender.host must be an IPv4 address') from exc
        if type(self.port) is not int or not MIN_PORT <= self.port <= MAX_PORT:
            raise ConfigurationError(f'sender.port must be an integer in {MIN_PORT}..{MAX_PORT}')
        if type(self.queue_capacity) is not int or self.queue_capacity <= 0:
            raise ConfigurationError('sender.queue_capacity must be a positive integer')
        for name in ('send_rate_hz', 'startup_timeout_s', 'shutdown_timeout_s'):
            object.__setattr__(self, name, _number(getattr(self, name), f'sender.{name}', positive=True))


def load_settings(path: str | Path) -> SenderSettings:
    try:
        data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ConfigurationError(f'Cannot read sender YAML {path}: {exc}') from exc
    if not isinstance(data, dict):
        raise ConfigurationError('sender configuration must be a mapping')
    unknown = data.keys() - {setting.name for setting in fields(SenderSettings)}
    if unknown:
        raise ConfigurationError(f'sender configuration has unknown keys: {sorted(map(str, unknown))}')
    return SenderSettings(**data)
