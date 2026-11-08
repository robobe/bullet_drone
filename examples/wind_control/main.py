"""Run a rendered takeoff with an example-owned wind editor."""
import sys

import click
from loguru import logger

from bullet_drone.config import ConfigurationError
from examples.altitude_hold_external_render.demo_cli import CliParser


EXIT_SUCCESS = 0
EXIT_FAILURE = 1
EXIT_USAGE_ERROR = 2
EXIT_INTERRUPTED = 130
WORLD_Z_AXIS = 2


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
    logger.add(sys.stderr, level='INFO', format='{level}: {message}', diagnose=False)
    try:
        from examples.wind_control.demo import run_demo
        result = run_demo(options.renderer, options.render_config)
        state = result.observation.ground_truth
        logger.info('Completed wind-control flight in {:.3f} s; final altitude {:.4f} m',
                    state.simulation_time_s, state.position_m[WORLD_Z_AXIS])
        return EXIT_SUCCESS
    except ConfigurationError as error:
        click.UsageError(str(error)).show()
        return EXIT_USAGE_ERROR
    except KeyboardInterrupt:
        logger.info('Interrupted; simulation and renderer closed')
        return EXIT_INTERRUPTED
    except Exception:
        logger.exception('External-render altitude hold failed')
        return EXIT_FAILURE


if __name__ == '__main__':
    raise SystemExit(main())
