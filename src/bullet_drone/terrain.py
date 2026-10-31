"""Validated shared float32 terrain heightmaps."""

from pathlib import Path
import struct

from bullet_drone.config import ConfigurationError, TerrainSettings


def load_heightmap(settings: TerrainSettings) -> list[float] | None:
    if settings.heightmap_path is None:
        return None
    path = Path(settings.heightmap_path).expanduser().resolve()
    expected = settings.samples_x * settings.samples_z
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise ConfigurationError(f"Cannot read terrain heightmap {path}: {exc}") from exc
    if len(raw) != expected * 4:
        raise ConfigurationError(f"Terrain heightmap must contain {expected} float32 samples")
    values = list(struct.unpack(f"<{expected}f", raw))
    if not all(value == value and abs(value) != float("inf") for value in values):
        raise ConfigurationError("Terrain heightmap contains non-finite samples")
    return values
