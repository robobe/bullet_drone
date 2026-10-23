"""Parse square example options without starting physics or configuring logging."""

from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import click
from tools.zmq_render import DEFAULT_RENDER_CONFIG


@dataclass(frozen=True, slots=True)
class DemoOptions:
    headless: bool = False
    telemetry_config: Path | None = None
    render_config: Path = DEFAULT_RENDER_CONFIG


class CliParser:
    def parse(self, argv: Sequence[str] | None = None) -> DemoOptions | int:
        @click.command()
        @click.option('--headless', is_flag=True, help='Run without the PyBullet GUI; controlled physics remains paced.')
        @click.option('--telemetry-config', type=click.Path(path_type=Path),
                      help='Enable the PlotJuggler sender using this YAML configuration.')
        @click.option('--render-config', default=DEFAULT_RENDER_CONFIG, type=click.Path(exists=True, path_type=Path),
                      help='Shared camera/control profile; starts stopped and waits for Godot.')
        def command(headless: bool, telemetry_config: Path | None, render_config: Path | None) -> DemoOptions:
            """Fly a settled 3 m square at altitude 3 m in RC angle mode."""
            return DemoOptions(headless, telemetry_config, render_config)

        return command.main(args=argv, prog_name='position-square', standalone_mode=False)
