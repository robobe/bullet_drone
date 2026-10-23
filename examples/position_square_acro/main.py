"""Square example entry point; importing this module has no startup effects."""

from collections.abc import Sequence
import sys

import click
from loguru import logger

from bullet_drone.config import ConfigurationError
from examples.position_square_acro.demo_cli import CliParser


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
    logger.add(sys.stderr, level='INFO', format='{level}: {message}', diagnose=False)
    renderer = None
    control = None
    try:
        from examples.position_square_acro.demo import run_demo

        from tools.zmq_render.publisher import RenderPublisher
        from tools.zmq_render.control import SimulationControl
        renderer = RenderPublisher(options.render_config)
        renderer.start()
        control = SimulationControl(renderer.transport_path, renderer)
        result = run_demo(gui=not options.headless, pace=True,
                          on_observation=renderer.consume, control=control)
        state = result.observation.ground_truth
        logger.info('Completed Acro square in {:.3f} s; final position {} m', state.simulation_time_s, state.position_m)
        return EXIT_SUCCESS
    except ConfigurationError as exc:
        click.UsageError(str(exc)).show()
        return EXIT_USAGE_ERROR
    except KeyboardInterrupt:
        logger.info('Simulation interrupted; resources closed')
        return EXIT_INTERRUPTED
    except Exception:
        logger.exception('Acro position square failed')
        return EXIT_FAILURE
    finally:
        if control is not None:
            control.close()
        if renderer is not None:
            renderer.close()


if __name__ == '__main__':
    raise SystemExit(main())
