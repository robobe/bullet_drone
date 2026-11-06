"""Standalone entry point; parsing/imports do not start firmware or physics."""
import sys

import click
from loguru import logger

from examples.altitde_hold_bt.demo_cli import CliParser


EXIT_SUCCESS = 0
EXIT_FAILURE = 1
EXIT_INTERRUPTED = 130


def main(argv=None):
    try:
        options = CliParser().parse(argv)
    except click.ClickException as error:
        error.show()
        return error.exit_code
    except (click.Abort, KeyboardInterrupt):
        return EXIT_INTERRUPTED
    if isinstance(options, int):
        return options
    logger.remove()
    logger.add(sys.stderr, level='INFO', colorize=sys.stderr.isatty(), diagnose=False,
               format='<green>{time:HH:mm:ss.SSS}</green> | <level>{level: <8}</level> | {message}')
    try:
        from examples.altitde_hold_bt.demo import run_demo
        result = run_demo(gui=not options.headless, sitl_binary=options.sitl_binary)
        logger.info('Completed Betaflight altitude hold in {:.3f} s; final altitude {:.4f} m',
                    result.observation.ground_truth.simulation_time_s,
                    result.observation.ground_truth.position_m[2])
        return EXIT_SUCCESS
    except KeyboardInterrupt:
        logger.info('Interrupted; firmware, MSP and physics closed')
        return EXIT_INTERRUPTED
    except Exception:
        logger.exception('Betaflight altitude hold failed')
        return EXIT_FAILURE


if __name__ == '__main__':
    raise SystemExit(main())
