"""Own a rendered altitude-hold mission and a separate SHM camera window."""
import subprocess
import sys

import click
from loguru import logger

from examples.altitude_hold_external_render.demo_cli import CliParser


CHILD_STOP_TIMEOUT_S = 3
EXIT_FAILURE = 1
EXIT_INTERRUPTED = 130


def run_example(options):
    from examples.altitude_hold_external_render.demo import run_demo

    process = subprocess.Popen([sys.executable, '-m', 'examples.takeoff_image_viewer.viewer',
                                '--render-config', str(options.render_config.resolve())])
    try:
        def check_viewer(observation):
            status = process.poll()
            if status is not None:
                raise RuntimeError(f'Camera viewer closed before mission completion (exit {status})')
        return run_demo(options.renderer, options.render_config, on_observation=check_viewer)
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=CHILD_STOP_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=CHILD_STOP_TIMEOUT_S)


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
    logger.add(sys.stderr, level='INFO', diagnose=False)
    try:
        result = run_example(options)
        logger.info('Takeoff, hold and landing complete at {:.3f} s',
                    result.observation.ground_truth.simulation_time_s)
        return 0
    except KeyboardInterrupt:
        logger.info('Interrupted; simulation, renderer and camera viewer closed')
        return EXIT_INTERRUPTED
    except Exception:
        logger.exception('Rendered takeoff failed')
        return EXIT_FAILURE


if __name__ == '__main__':
    raise SystemExit(main())
