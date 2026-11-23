"""Red-truck visual-approach entry point."""
from collections.abc import Sequence
from contextlib import ExitStack
import sys

import click
from loguru import logger

from examples.red_truck_visual.demo_cli import CliParser


EXIT_SUCCESS = 0
EXIT_FAILURE = 1
EXIT_USAGE_ERROR = 2
EXIT_INTERRUPTED = 130


def main(argv: Sequence[str] | None = None) -> int:
    try:
        options = CliParser().parse(argv)
    except click.ClickException as exc:
        exc.show()
        return exc.exit_code
    except (click.Abort, KeyboardInterrupt):
        click.echo('Simulation interrupted.', err=True)
        return EXIT_INTERRUPTED
    if isinstance(options, int):
        return options
    logger.remove()
    logger.add(sys.stderr, level='INFO', colorize=sys.stderr.isatty(), diagnose=False, enqueue=True,
               format='<green>{time:HH:mm:ss.SSS}</green> | <level>{level:8}</level> | <level>{message}</level>')
    try:
        from examples.red_truck_visual.demo import run_demo
        result = run_demo(renderer=options.renderer, render_config=options.render_config,
                          sitl_binary=options.sitl_binary)
        logger.info('Completed red-truck approach at {:.3f} simulated seconds',
                    result.observation.ground_truth.simulation_time_s)
        return EXIT_SUCCESS
    except KeyboardInterrupt:
        logger.info('Interrupted; renderer, firmware and physics closed')
        return EXIT_INTERRUPTED
    except Exception:
        logger.exception('Red-truck visual approach failed')
        return EXIT_FAILURE


if __name__ == '__main__':
    raise SystemExit(main())
