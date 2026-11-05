"""Parse example options without opening physics or configuring logging."""

from collections.abc import Sequence
from dataclasses import dataclass

import click


@dataclass(frozen=True, slots=True)
class DemoOptions:
    headless: bool = False


class CliParser:
    def parse(self, argv: Sequence[str] | None = None) -> DemoOptions | int:
        @click.command()
        @click.option('--headless', is_flag=True, help='Run without the PyBullet GUI or wall-clock pacing.')
        def command(headless: bool) -> DemoOptions:
            """Take off to 3 m, hold for 10 s, then land in RC angle mode."""
            return DemoOptions(headless)

        return command.main(args=argv, prog_name='altitude-hold', standalone_mode=False)
