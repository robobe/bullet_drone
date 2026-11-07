"""Reuse terrain mission with a telemetry-enabled SITL session."""
from examples.terrain_following_bt.demo import run_demo as run_terrain
from examples.terrain_following_mavlink.cli import DEFAULT_SITL_BINARY
from examples.terrain_following_mavlink.sitl import SitlSession


def run_demo(*, sitl_binary=DEFAULT_SITL_BINARY, on_observation=None):
    return run_terrain(sitl_binary=sitl_binary, on_observation=on_observation,
                       sitl_session_factory=SitlSession)
