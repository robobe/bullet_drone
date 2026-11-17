"""Launcher options without starting simulation resources."""
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import click


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SITL_BINARY = PROJECT_ROOT / 'bin/betaflight_SITL.elf'
DEFAULT_RENDER_CONFIG = PROJECT_ROOT / 'external_render/configs/physics.json'
DEFAULT_RENDERER = PROJECT_ROOT / 'external_render/dist/linux-x86_64/render'


@dataclass(frozen=True, slots=True)
class Options:
    sitl: bool = True
    render: bool = True
    sitl_binary: Path = DEFAULT_SITL_BINARY
    render_config: Path = DEFAULT_RENDER_CONFIG
    renderer: Path = DEFAULT_RENDERER


class CliParser:
    def parse(self, argv: Sequence[str] | None = None) -> Options | int:
        @click.command()
        @click.option('--sitl/--no-sitl', default=True, help='Start Betaflight SITL (external controller owns MSP).')
        @click.option('--render/--no-render', default=True, help='Start the Godot renderer.')
        @click.option('--sitl-binary', type=click.Path(dir_okay=False, path_type=Path), default=DEFAULT_SITL_BINARY)
        @click.option('--render-config', type=click.Path(exists=True, dir_okay=False, path_type=Path),
                      default=DEFAULT_RENDER_CONFIG)
        @click.option('--renderer', type=click.Path(dir_okay=False, path_type=Path), default=DEFAULT_RENDERER)
        def command(sitl, render, sitl_binary, render_config, renderer):
            """Run a persistent PyBullet simulation with optional SITL and rendering."""
            return Options(sitl, render, sitl_binary, render_config, renderer)

        return command.main(args=argv, prog_name='simulation-launcher', standalone_mode=False)
