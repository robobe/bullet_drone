"""Supervise a persistent PyBullet loop and optional external processes."""
from collections.abc import Sequence
from contextlib import ExitStack
import sys
import time

import click
from loguru import logger

from bullet_drone.betaflight.backend import BetaflightBackend
from bullet_drone.betaflight.bridge import UdpSitlBridge
from bullet_drone.config import ControlSettings, DroneSettings, SimulationConfig, SimulationSettings
from bullet_drone.environment import DroneEnvironment
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.sensors import BarometerSensor, ImuSensor
from bullet_drone.vehicle import load_racer_parameters
from examples.simulation_launcher.cli import CliParser
from examples.terrain_following_bt.demo import GRAVITY_M_S2, GRAVITY_VECTOR_M_S2, INITIAL_POSITION_M
from examples.terrain_following_bt.sitl import SitlSession


RATE_HZ = 240
TIMESTEP_S = 1 / RATE_HZ
MAX_SIMULATION_STEPS = 2**31 - 1


def run(options):
    with ExitStack() as resources:
        if options.sitl:
            sitl = resources.enter_context(SitlSession(options.sitl_binary, connect_msp=False))
        viewer = None
        if options.render:
            from external_render.renderer import ExternalRenderer
            viewer = resources.enter_context(ExternalRenderer(
                options.renderer, options.render_config, profile_name='Simulation Launcher',
                backend='betaflight' if options.sitl else 'internal'))
        vehicle = load_racer_parameters()
        config = SimulationConfig(SimulationSettings(TIMESTEP_S, MAX_SIMULATION_STEPS,
                                                     GRAVITY_VECTOR_M_S2, gui=False),
                                  DroneSettings(1.0, INITIAL_POSITION_M),
                                  control=ControlSettings('betaflight' if options.sitl else 'internal'))
        physics = PyBulletPhysics(config, vehicle)
        resources.callback(physics.close)
        environment = DroneEnvironment(
            physics, imu=ImuSensor(GRAVITY_VECTOR_M_S2, config.sensors.imu, sample_period_s=TIMESTEP_S),
            barometer=BarometerSensor(config.sensors.barometer))
        resources.callback(environment.close)
        backend = BetaflightBackend(vehicle, UdpSitlBridge()) if options.sitl else None
        if backend is not None:
            resources.callback(backend.close)
        observation = environment.reset()
        if backend is not None:
            backend.reset(observation)
        ready = False
        next_step = time.monotonic()
        while True:
            if options.sitl:
                sitl.check_running()
            if viewer:
                viewer.ensure_running()
                if not ready:
                    viewer.wait_ready(observation)
                    viewer.set_running(True)
                    ready = True
            remaining = next_step - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            next_step = time.monotonic() + TIMESTEP_S
            if backend:
                observation = environment.step(backend.command(observation))
            else:
                observation = environment.hold()
            if viewer:
                viewer.consume(observation)


def main(argv: Sequence[str] | None = None) -> int:
    try:
        options = CliParser().parse(argv)
    except click.ClickException as error:
        error.show()
        return error.exit_code
    except (click.Abort, KeyboardInterrupt):
        return 130
    if isinstance(options, int):
        return options
    logger.remove()
    logger.add(sys.stderr, level='INFO', colorize=sys.stderr.isatty(),
               format='<green>{time:HH:mm:ss.SSS}</green> | <level>{level:8}</level> | <level>{message}</level>',
               diagnose=False)
    try:
        run(options)
    except KeyboardInterrupt:
        logger.info('Interrupted; simulation, renderer, and SITL closed')
        return 130
    except Exception:
        logger.exception('Simulation launcher failed')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
