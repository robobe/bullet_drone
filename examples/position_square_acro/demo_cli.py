"""Parse options without starting firmware, rendering, physics, or logging."""

from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import click


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SITL_BINARY = PROJECT_ROOT / '.sitl/betaflight-source/obj/main/betaflight_SITL.elf'
DEFAULT_RENDER_CONFIG = PROJECT_ROOT / 'external_render/configs/physics.json'
DEFAULT_RENDERER = PROJECT_ROOT / 'external_render/dist/linux-x86_64/render'


@dataclass(frozen=True, slots=True)
class DemoOptions:
    headless: bool = False
    sitl_binary: Path = DEFAULT_SITL_BINARY
    render_config: Path = DEFAULT_RENDER_CONFIG
    renderer: Path = DEFAULT_RENDERER


class CliParser:
    def parse(self, argv: Sequence[str] | None = None) -> DemoOptions | int:
        @click.command()
        @click.option('--headless', is_flag=True,
                      help='Run automatically without the external viewer; SITL remains paced.')
        @click.option('--sitl-binary', default=DEFAULT_SITL_BINARY,
                      type=click.Path(dir_okay=False, path_type=Path),
                      help='Prebuilt Betaflight 2026.6.2 SITL executable.')
        @click.option('--render-config', default=DEFAULT_RENDER_CONFIG,
                      type=click.Path(exists=True, dir_okay=False, path_type=Path),
                      help='Camera/transport profile for the automatic external viewer.')
        @click.option('--renderer', default=DEFAULT_RENDERER,
                      type=click.Path(dir_okay=False, path_type=Path),
                      help='Exported Godot viewer executable (ignored with --headless).')
        def command(headless, sitl_binary, render_config, renderer):
            """Fly a 3 m square using Betaflight SITL in Acro mode."""
            return DemoOptions(headless, sitl_binary, render_config, renderer)

        return command.main(args=argv, prog_name='position-square-acro', standalone_mode=False)
