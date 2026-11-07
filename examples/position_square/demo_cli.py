"""Parse square example options without starting physics or configuring logging."""

from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import click


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RENDER_CONFIG = PROJECT_ROOT / 'external_render/configs/physics.json'
DEFAULT_RENDERER = PROJECT_ROOT / 'external_render/dist/linux-x86_64/render'


@dataclass(frozen=True, slots=True)
class DemoOptions:
    headless: bool = False
    telemetry_config: Path | None = None
    render_config: Path = DEFAULT_RENDER_CONFIG
    renderer: Path = DEFAULT_RENDERER
    profile: Path | None = None


class CliParser:
    def parse(self, argv: Sequence[str] | None = None) -> DemoOptions | int:
        @click.command()
        @click.option('--headless', is_flag=True, help='Run automatically without windows or rendering dependencies.')
        @click.option('--telemetry-config', type=click.Path(path_type=Path),
                      help='Enable the PlotJuggler sender using this YAML configuration.')
        @click.option('--render-config', default=DEFAULT_RENDER_CONFIG, type=click.Path(exists=True, path_type=Path),
                      help='Camera/transport profile for the automatic external viewer.')
        @click.option('--renderer', default=DEFAULT_RENDERER, type=click.Path(dir_okay=False, path_type=Path),
                      help='Exported Godot viewer executable (ignored with --headless).')
        @click.option('--profile', type=click.Path(exists=True, dir_okay=False, path_type=Path),
                      help='Simulation YAML profile, including its drone URDF.')
        def command(headless: bool, telemetry_config: Path | None, render_config: Path, renderer: Path,
                    profile: Path | None) -> DemoOptions:
            """Fly a settled 3 m square at altitude 3 m in RC angle mode."""
            return DemoOptions(headless, telemetry_config, render_config, renderer, profile)

        return command.main(args=argv, prog_name='position-square', standalone_mode=False)
