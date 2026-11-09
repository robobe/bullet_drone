"""Parse options without starting firmware, physics or logging."""
from dataclasses import dataclass
from pathlib import Path

import click


DEFAULT_SITL_BINARY = Path(__file__).resolve().parents[2] / 'bin/betaflight_SITL.elf'


@dataclass(frozen=True)
class DemoOptions:
    headless: bool = False
    sitl_binary: Path = DEFAULT_SITL_BINARY


class CliParser:
    def parse(self, argv=None):
        @click.command()
        @click.option('--headless', is_flag=True, help='Disable the PyBullet window; SITL remains paced.')
        @click.option('--sitl-binary', default=DEFAULT_SITL_BINARY,
                      type=click.Path(dir_okay=False, path_type=Path), help='Prebuilt Betaflight 2026.6.2 SITL executable.')
        def command(headless, sitl_binary):
            """Take off to 3 m, hold for 10 s and land using Betaflight SITL."""
            return DemoOptions(headless, sitl_binary)
        return command.main(args=argv, prog_name='altitde-hold-bt', standalone_mode=False)
