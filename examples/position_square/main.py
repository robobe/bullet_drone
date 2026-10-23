"""Square example entry point; importing this module has no startup effects."""

from collections.abc import Sequence
import sys

import click
from loguru import logger

from bullet_drone.config import ConfigurationError
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
    logger.add(sys.stderr, level='INFO', format='{level}: {message}', diagnose=False)
    sender = None
    renderer = None
    control = None
    try:
        from examples.position_square.demo import run_demo

        if options.telemetry_config is not None:
            from tools.telemetry_sender.config import load_settings
            from tools.telemetry_sender.sender import TelemetrySender

            settings = load_settings(options.telemetry_config)
            sender = TelemetrySender(settings)
            sender.start()
        if options.render_config is not None:
            from tools.zmq_render.publisher import RenderPublisher
            renderer = RenderPublisher(options.render_config)
            renderer.start()
        from tools.zmq_render.control import SimulationControl
        control = SimulationControl(renderer.transport_path, renderer)
        consumers = [consumer for consumer in (sender, renderer) if consumer is not None]
        def consume(observation):
            for consumer in consumers:
                consumer.consume(observation)
        kwargs = {'control': control, 'gui': not options.headless, 'on_observation': consume if consumers else None}
        if renderer is not None:
            kwargs['pace'] = True
        result = run_demo(**kwargs)
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
    finally:
        if control is not None:
            control.close()
        if renderer is not None:
            renderer.close()
        if sender is not None:
            sender.close()


if __name__ == '__main__':
    raise SystemExit(main())
