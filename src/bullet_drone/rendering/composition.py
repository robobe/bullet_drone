"""Resolve modes before allocating physics or transport resources."""
from bullet_drone.config import ConfigurationError
from bullet_drone.rendering.config import load_render_settings
from bullet_drone.rendering.consumer import RenderConsumer
from bullet_drone.rendering.models import SnapshotBuilder
from bullet_drone.rendering.udp import UdpRenderer


def compose_rendering(renderer=None, render_config=None, *, gui=False, headless=False):
    if render_config is not None and renderer != 'godot':
        raise ConfigurationError("render_config requires renderer='godot'")
    if headless and renderer in ('godot', 'pybullet'):
        raise ConfigurationError('headless conflicts with a visible renderer')
    if renderer == 'godot':
        settings = load_render_settings(render_config)
        return False, True, (RenderConsumer(SnapshotBuilder(settings), UdpRenderer(settings), settings.rate_hz),)
    selected_gui = renderer == 'pybullet' if renderer is not None else gui and not headless
    return selected_gui, selected_gui, ()
