"""Square example entry point; importing this module has no startup effects."""

from collections.abc import Sequence
from contextlib import ExitStack
import sys

import click
from loguru import logger

from bullet_drone.config import ConfigurationError, YamlConfigLoader
from examples.position_square.demo_cli import CliParser


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
               format='{level}: {message}', diagnose=False)
    try:
        from examples.position_square.demo import run_demo

        with ExitStack() as resources:
            viewer = None
            if not options.headless:
                from external_render.renderer import ExternalRenderer

                profile_name = options.profile.stem if options.profile is not None else 'position_square'
                backend = (YamlConfigLoader().load(options.profile).control.backend
                           if options.profile is not None else 'internal')
                viewer = resources.enter_context(ExternalRenderer(
                    options.renderer, options.render_config, profile_name=profile_name, backend=backend))
            ready = False

            def consume(observation):
                nonlocal ready
                if viewer is not None:
                    if not ready:
                        viewer.wait_ready(observation)
                        viewer.set_state(observation, 'running', 'automatic_start')
                        ready = True
                    viewer.consume(observation)
            result = run_demo(gui=False, pace=viewer is not None, profile=options.profile,
                              on_observation=consume if viewer is not None else None)
            if viewer is not None:
                viewer.set_state(result.observation, 'stopped', 'completed')
            state = result.observation.ground_truth
            logger.info('Completed square in {:.3f} s; final position {} m', state.simulation_time_s, state.position_m)
            return EXIT_SUCCESS
    except ConfigurationError as exc:
        click.UsageError(str(exc)).show()
        return EXIT_USAGE_ERROR
    except KeyboardInterrupt:
        logger.info('Simulation interrupted; resources closed')
        return EXIT_INTERRUPTED
    except Exception:
        logger.exception('Position square failed')
        return EXIT_FAILURE


if __name__ == '__main__':
    raise SystemExit(main())
