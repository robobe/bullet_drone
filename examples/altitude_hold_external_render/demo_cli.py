"""Parse arguments without starting graphics, physics or logging."""
from dataclasses import dataclass
from pathlib import Path

import click


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RENDERER = PROJECT_ROOT / 'external_render/dist/linux-x86_64/render'
DEFAULT_RENDER_CONFIG = PROJECT_ROOT / 'external_render/configs/physics.json'


@dataclass(frozen=True, slots=True)
class DemoOptions:
    renderer: Path = DEFAULT_RENDERER
    render_config: Path = DEFAULT_RENDER_CONFIG


class CliParser:
    def parse(self, argv=None):
        @click.command()
        @click.option('--renderer', default=DEFAULT_RENDERER,
                      type=click.Path(dir_okay=False, path_type=Path),
                      help='Packaged renderer launcher. Build it with external_render/build.py.')
        @click.option('--render-config', default=DEFAULT_RENDER_CONFIG,
                      type=click.Path(exists=True, dir_okay=False, path_type=Path),
                      help='Camera/transport profile for the external renderer.')
        def command(renderer, render_config):
            """Launch the external viewer, climb to 3 m, hold for 10 s, then land."""
            return DemoOptions(renderer, render_config)
        return command.main(args=argv, prog_name='altitude-hold-external-render', standalone_mode=False)
