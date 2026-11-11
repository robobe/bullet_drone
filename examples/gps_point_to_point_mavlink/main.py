"""Betaflight SITL GPS point-to-point example entry point."""

from collections.abc import Sequence
from contextlib import ExitStack
import sys
from pathlib import Path

import click
from loguru import logger

from bullet_drone.config import ConfigurationError
from examples.gps_point_to_point_mavlink.cli import CliParser


PROJECT_ROOT = Path(__file__).resolve().parents[2]
GPS_ORIGIN = (32.0, 35.0, 100.0)
START_GPS = (31.998941979585357, 35.001727163237504)
DESTINATION_GPS = (32.00074562062305, 34.998061181190316)
TERRAIN_PATH = PROJECT_ROOT / 'custom/terrain/terrain_heightmap.f32'

MAX_LANDING_ERROR_M = 0.5
MAX_CROSS_TRACK_ERROR_M = 0.5
MIN_CRUISE_CLEARANCE_M = 3.0

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
        from examples.gps_point_to_point_mavlink.demo import run_demo

        with ExitStack() as resources:
            viewer = None
            if not options.headless:
                from external_render.renderer import ExternalRenderer

                viewer = resources.enter_context(ExternalRenderer(
                    options.renderer, options.render_config, profile_name='GPS Point to Point',
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

            result = run_demo(origin=GPS_ORIGIN, start_gps=START_GPS, end_gps=DESTINATION_GPS,
                              terrain_path=TERRAIN_PATH, sitl_binary=options.sitl_binary,
                              on_observation=render if viewer is not None else None)
            if viewer is not None:
                viewer.set_state(result.observation, 'stopped', 'completed')
            state = result.observation.ground_truth
            logger.info('Completed GPS point-to-point flight in {:.3f} s; final position {} m',
                        state.simulation_time_s, state.position_m)
            logger.info('Validation: landing_error={:.3f} m cross_track={:.3f} m minimum_clearance={:.3f} m contact={} armed={}',
                        result.landing_error_m, result.max_cross_track_m, result.minimum_clearance_m,
                        result.contact, result.armed)
            if (result.landing_error_m > MAX_LANDING_ERROR_M
                    or result.max_cross_track_m > MAX_CROSS_TRACK_ERROR_M
                    or result.minimum_clearance_m < MIN_CRUISE_CLEARANCE_M
                    or not result.contact or result.armed):
                raise RuntimeError('Independent flight verification failed')
            return EXIT_SUCCESS
    except ConfigurationError as exc:
        click.UsageError(str(exc)).show()
        return EXIT_USAGE_ERROR
    except KeyboardInterrupt:
        logger.info('Interrupted; renderer, firmware, MSP and physics closed')
        return EXIT_INTERRUPTED
    except Exception:
        logger.exception('Betaflight GPS point-to-point failed')
        return EXIT_FAILURE


if __name__ == '__main__':
    raise SystemExit(main())
