"""Optional application-side renderer and simulation control adapters."""
from pathlib import Path

DEFAULT_RENDER_CONFIG = Path(__file__).resolve().parents[3] / 'bullet_render/configs/physics.json'
