"""Parse square example options without starting physics or configuring logging."""

from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import click
from tools.zmq_render import DEFAULT_RENDER_CONFIG


@dataclass(frozen=True, slots=True)
class DemoOptions:
    headless: bool = False
    render_config: Path = DEFAULT_RENDER_CONFIG


class CliParser:
    def parse(self, argv: Sequence[str] | None = None) -> DemoOptions | int:
        @click.command()
        @click.option('--headless', is_flag=True, help='Run without the PyBullet GUI; controlled physics remains paced.')
        @click.option('--render-config', default=DEFAULT_RENDER_CONFIG, type=click.Path(exists=True, path_type=Path),
                      help='Shared camera/control profile; starts stopped and waits for Godot.')
        def command(headless: bool, render_config: Path) -> DemoOptions:
            """Fly a settled 3 m square at altitude 3 m in RC Acro mode."""
            return DemoOptions(headless, render_config)

        return command.main(args=argv, prog_name='position-square-acro', standalone_mode=False)
