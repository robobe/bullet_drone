"""Betaflight SITL terrain-following example entry point."""

from collections.abc import Sequence
from contextlib import ExitStack
import sys

import click
from loguru import logger

from bullet_drone.config import ConfigurationError
from examples.terrain_following_mavlink.cli import CliParser


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
    logger.add(sys.stderr, level='INFO', colorize=sys.stderr.isatty(),
               format='<green>{time:HH:mm:ss.SSS}</green> | <level>{level:8}</level> | <level>{message}</level>',
               diagnose=False)
    try:
        from examples.terrain_following_mavlink.demo import run_demo

        with ExitStack() as resources:
            viewer = None
            if not options.headless:
                from external_render.renderer import ExternalRenderer

                viewer = resources.enter_context(ExternalRenderer(
                    options.renderer, options.render_config, profile_name='Terrain Following MAVLink',
                    backend='betaflight'))
            ready = False

            def render(observation):
                nonlocal ready
                if viewer is None:
                    return
                if not ready:
                    viewer.wait_ready(observation)
                    viewer.set_state(observation, 'running', 'automatic_start')
                    ready = True
                viewer.consume(observation)

            result = run_demo(sitl_binary=options.sitl_binary,
                              on_observation=render if viewer is not None else None)
            if viewer is not None:
                viewer.set_state(result.observation, 'stopped', 'completed')
            state = result.observation.ground_truth
            logger.info('Completed terrain-following flight in {:.3f} s; final position {} m',
                        state.simulation_time_s, state.position_m)
            return EXIT_SUCCESS
    except ConfigurationError as exc:
        click.UsageError(str(exc)).show()
        return EXIT_USAGE_ERROR
    except KeyboardInterrupt:
        logger.info('Interrupted; renderer, firmware, MSP and physics closed')
        return EXIT_INTERRUPTED
    except Exception:
        logger.exception('Betaflight terrain following failed')
        return EXIT_FAILURE


if __name__ == '__main__':
    raise SystemExit(main())
