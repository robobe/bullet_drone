"""Godot lifecycle with app-owned expiring RC and selectable sensor pose."""
from dataclasses import asdict, replace
import json
import sys
import time
from pathlib import Path

import click
from loguru import logger
from bullet_msgs import load_transport
from bullet_msgs.provenance import revisions
from bullet_drone.config import ConfigurationError, DroneSettings, SimulationConfig, SimulationSettings, WindSettings, YamlConfigLoader
from bullet_drone.pose import resolve_pose
from bullet_drone.simulation import RcSimulation
from bullet_drone.commands import RcInput
from bullet_drone.vehicle import load_racer_parameters
from examples.position_square.demo import FLIGHT_CONTROL_SETTINGS
from tools.zmq_render import DEFAULT_RENDER_CONFIG
from tools.zmq_render.control import SimulationControl
from tools.zmq_render.spawn import SpawnSelection
from tools.zmq_render.wind import WindControl
from tools.zmq_render.scenario import ScenarioTermination
from tools.zmq_render.flight import FlightLease, FlightReceiver
from tools.zmq_render.publisher import RenderPublisher


PHYSICS_HZ = 240
FEEDBACK_HZ = 120
COMMAND_HZ = 100
MAX_EPISODE_STEPS = 10**9
NANOSECONDS_PER_MICROSECOND = 1000
FIRST_COMMAND_TIMEOUT_S = 0.5
GRAVITY_M_S2 = 9.81
MASS_KG = 1.0
INITIAL_POSITION_M = (0.0, 0.0, 0.015)


class CliParser:
    def parse(self, argv=None):
        @click.command()
        @click.option('--render-config', default=DEFAULT_RENDER_CONFIG, type=click.Path(exists=True, path_type=Path))
        @click.option('--config', type=click.Path(exists=True, path_type=Path), help='Physics YAML, including sensor noise and pose sources.')
        def command(render_config, config):
            return render_config, config
        return command.main(args=argv, prog_name='external-rc', standalone_mode=False)


def _close_spawn_simulation(simulation):
    """A cleanup failure must not discard the retained or newly initialized world."""
    try:
        simulation.close()
    except Exception:
        logger.exception('Could not close retired spawn simulation')


def run(path, *, config_path=None, simulation_factory=RcSimulation):
    config = (YamlConfigLoader().load(config_path) if config_path is not None else
              SimulationConfig(SimulationSettings(1 / PHYSICS_HZ, MAX_EPISODE_STEPS, (0., 0., -GRAVITY_M_S2), gui=False),
                               DroneSettings(MASS_KG, INITIAL_POSITION_M)))
    # Godot owns the viewer; sensor and physics settings still come from YAML.
    config = replace(config, simulation=replace(config.simulation, gui=False))
    gravity = config.simulation.gravity_m_s2
    if gravity[:2] != (0., 0.) or gravity[2] >= 0:
        raise ConfigurationError('External RC requires downward world-Z gravity')
    physics_hz = 1 / config.simulation.timestep_s
    renderer = RenderPublisher(path, pose_sources=config.pose_sources)
    control = receiver = simulation = None
    try:
        renderer.start()
        transport = load_transport(renderer.transport_path)
        scenario = ScenarioTermination()
        wind = WindControl()
        control = SimulationControl(renderer.transport_path, renderer, scenario=scenario, spawn=SpawnSelection(config.drone.initial_position_m), wind=wind)
        lease = FlightLease(control.physics_id)
        receiver = FlightReceiver(transport['flight_command_endpoint'], lease)
        vehicle = load_racer_parameters()
        platform = dict(schema_version=1, physics_id=control.physics_id, stream_id=renderer.stream,
                        mass_kg=config.drone.mass_kg, gravity_m_s2=-gravity[2],
                        max_thrust_n=len(vehicle.rotor_positions_m) * vehicle.max_thrust_per_rotor_n,
                        rc_angle_scale_rad=config.rc.max_angle_rad,
                        rc_yaw_rate_scale_rad_s=FLIGHT_CONTROL_SETTINGS.max_rates_rad_s[2],
                        aetr_channels=list(config.rc.aetr_channels),
                        arm_channel=config.rc.arm_channel, mode_channel=config.rc.mode_channel,
                        camera=renderer.config['camera'], attitude_model=('imu_complementary' if config.pose_sources.attitude == 'imu' else 'ideal'),
                        physics_hz=physics_hz, feedback_hz=FEEDBACK_HZ, command_hz=COMMAND_HZ)
        click.echo('=== bullet_drone loaded configuration ===\n' + json.dumps({
            'physics_config_path': str(Path(config_path).resolve()) if config_path is not None else None,
            'simulation': asdict(config),
            'flight_control': asdict(FLIGHT_CONTROL_SETTINGS),
            'vehicle': asdict(vehicle),
            'render_config_path': str(Path(path).resolve()),
            'render': renderer.config,
            'transport_config_path': str(renderer.transport_path.resolve()),
            'transport': transport,
            'platform': platform,
        }, indent=2))
        source_revisions = revisions(Path(__file__).resolve().parents[3])
        def configure_analysis():
            renderer.set_analysis_config(dict(simulation=asdict(config), vehicle=asdict(vehicle),
                                             flight_control=asdict(FLIGHT_CONTROL_SETTINGS), render=renderer.config,
                                             transport=transport), source_revisions)
        configure_analysis()
        simulation = simulation_factory(config, FLIGHT_CONTROL_SETTINGS, vehicle)
        observation = simulation.reset()
        applied_sequence = 0
        armed, mode = False, 'angle'
        def publish(*, captured=False):
            nonlocal source
            state = resolve_pose(observation, config.pose_sources)
            renderer.set_wind(wind.packet(control, observation, simulation.wind))
            if captured:
                source = dict(physics_id=control.physics_id, episode_id=state.episode_id, stream_id=renderer.stream,
                              step_index=state.step_index, simulation_time_s=state.simulation_time_s,
                              measurement_us=time.monotonic_ns() // NANOSECONDS_PER_MICROSECOND)
                scenario.observe(source)
                renderer.consume_analysis(observation, source)
            reading = observation.sensors.barometer
            feedback = dict(schema_version=2, source=source, takeoff_height_m=control.spawn.takeoff_height_m, barometer_altitude_m=None if reading is None else reading.altitude_m,
                            orientation_xyzw=list(state.orientation_xyzw), armed=armed, mode=mode,
                            last_applied_sequence=applied_sequence, owner_id=lease.owner,
                            state=control.state, reason=control.reason)
            renderer.set_flight(platform, feedback)
            if captured:
                renderer.consume(observation, source=source)
        source = None
        control.transition('stopped', 'waiting_app', observation)
        publish(captured=True)
        start_time = first_deadline = time.monotonic()
        while True:
            receiver.poll(observation.ground_truth.episode_id)
            action = control.poll()
            if action == 'wind':
                try:
                    settings = WindSettings(**wind.pending['settings'])
                    simulation.set_wind(settings)
                except (ValueError, RuntimeError):
                    logger.exception('Wind update failed; retaining previous settings')
                    wind.finish(False, 'update_failed')
                else:
                    config = replace(config, wind=settings)
                    wind.finish(True, 'wind_applied')
                    configure_analysis()
                publish()
            elif action == 'spawn':
                candidate = None
                try:
                    request = control.spawn.pending
                    candidate_config = replace(config, drone=replace(config.drone, initial_position_m=tuple(request['position_m'])))
                    candidate = simulation_factory(candidate_config, FLIGHT_CONTROL_SETTINGS, vehicle)
                    candidate_observation = candidate.reset()
                except Exception:
                    logger.exception('Spawn initialization failed; retaining previous simulation')
                    if candidate is not None:
                        _close_spawn_simulation(candidate)
                    control.spawn.finish(False, 'initialization_failed')
                else:
                    previous = simulation
                    simulation, config, observation = candidate, candidate_config, candidate_observation
                    configure_analysis()
                    _close_spawn_simulation(previous)
                    lease.end()
                    scenario.reset()
                    applied_sequence, armed, mode = 0, False, 'angle'
                    control.spawn.finish(True, 'spawn_applied')
                    control.transition('stopped', 'spawn_applied', observation)
                    publish(captured=True)
                control.publish(observation, force=True)
            elif action == 'reset' or (action == 'start' and control.state != 'running'):
                can_start = action == 'start' and lease.ready()
                if action == 'start' and not can_start:
                    if scenario.result is None:
                        control.transition('stopped', 'app_not_ready', observation)
                    else:
                        control.publish(observation, force=True)
                else:
                    owner_packet = lease.candidate
                    lease.end()
                    scenario.reset()
                    observation = simulation.reset()
                    applied_sequence, armed, mode = 0, False, 'angle'
                    if can_start:
                        lease.candidate = owner_packet
                        lease.begin(observation.ground_truth.episode_id)
                    control.transition('running' if can_start else 'stopped', 'started' if can_start else 'reset', observation)
                    start_time = time.monotonic()
                    first_deadline = start_time + FIRST_COMMAND_TIMEOUT_S
                    publish(captured=True)
            elif action == 'collision':
                lease.end()
                control.transition('stopped', 'target_hit', observation)
                publish()
            elif action in ('stop', 'controller_timeout', 'controller_changed') and control.state == 'running':
                control.transition('stopped', action, observation)
                lease.end()
            control.publish(observation)
            if control.state != 'running':
                publish()
                continue
            receiver.poll(observation.ground_truth.episode_id)
            channels = lease.command()
            if channels is None:
                if lease.latest is None and time.monotonic() < first_deadline:
                    publish()
                    time.sleep(config.simulation.timestep_s)
                    start_time = time.monotonic()
                    continue
                control.transition('stopped', lease.reason, observation)
                lease.end()
                publish()
                continue
            observation = simulation.step(RcInput(tuple(channels)))
            applied_sequence = lease.sequence
            armed, mode = simulation.last_command.armed, simulation.last_command.mode
            publish(captured=True)
            remaining = start_time + observation.ground_truth.simulation_time_s - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
    finally:
        if simulation is not None:
            simulation.close()
        if receiver is not None:
            receiver.close()
        if control is not None:
            control.close()
        renderer.close()


def main(argv=None):
    logger.remove()
    logger.add(sys.stderr, level='INFO', diagnose=False)
    try:
        options = CliParser().parse(argv)
        if isinstance(options, int):
            return options
        path, config_path = options
        run(path, config_path=config_path)
    except KeyboardInterrupt:
        return 130
    except Exception:
        logger.exception('External RC example failed')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
