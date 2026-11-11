"""Example-owned simulation execution, terrain preflight and resources."""
from contextlib import ExitStack
from dataclasses import dataclass, replace
from functools import partial
from math import ceil, dist, hypot
import time

from loguru import logger
from bullet_drone.betaflight.backend import BetaflightBackend
from bullet_drone.betaflight.bridge import UdpSitlBridge
from bullet_drone.betaflight.fdm import pack_fdm
from bullet_drone.config import (ControlSettings, DroneSettings, GpsSettings, SensorsSettings,
                                SimulationConfig, SimulationSettings, TerrainSettings)
from bullet_drone.environment import DroneEnvironment
from bullet_drone.models import DroneObservation
from bullet_drone.physics import PyBulletPhysics
from bullet_drone.sensors import BarometerSensor, GpsSensor, ImuSensor
from bullet_drone.vehicle import load_racer_parameters
from .mission import Mission
from .navigation import GeographicFrame
from .sitl import SitlSession
from .telemetry import Telemetry

TIMESTEP_S = 1 / 240
MISSION_TIMEOUT_S = 300.0
STARTUP_TIMEOUT_S = 15.0
STATUS_PERIOD_S = 0.1
STATUS_TIMEOUT_S = 0.5
REPORT_PERIOD_S = 5.0
GRAVITY_M_S2 = 9.81
GRAVITY_VECTOR_M_S2 = (0.0, 0.0, -GRAVITY_M_S2)
MASS_KG = 1.0
TAKEOFF_HEIGHT_M = 10.0
VEHICLE_GROUND_OFFSET_M = 0.015
RAY_ORIGIN_Z_M = 500.0
RAY_LENGTH_M = 1000.0
CORRIDOR_HALF_WIDTH_M = 2.5
CORRIDOR_SAMPLE_SPACING_M = 0.5
MIN_CLEARANCE_M = 3.0
TERRAIN_WORLD_SIZE_M = 500.0
TERRAIN_SAMPLES = 121
TERRAIN_ORIGIN_M = (-250.0, -250.0)
MAX_ROUTE_RADIUS_M = 240.0
LANDING_PATCH_RADIUS_M = 0.3
MAX_LANDING_PATCH_HEIGHT_CHANGE_M = 0.05


@dataclass(frozen=True)
class DemoResult:
    observation: DroneObservation
    transitions: tuple
    max_cross_track_m: float
    minimum_clearance_m: float
    landing_error_m: float
    contact: bool
    armed: bool


def surface_height(physics, x, y):
    hit = physics.ray_surface((x, y, RAY_ORIGIN_Z_M), (0.0, 0.0, -1.0), RAY_LENGTH_M)
    if hit is None:
        raise ValueError(f'No mapped terrain at {(x, y)}')
    return RAY_ORIGIN_Z_M - hit.distance_m


def check_landing_patch(physics, point):
    x, y = point
    radius = LANDING_PATCH_RADIUS_M
    heights = [surface_height(physics, x + dx, y + dy)
               for dx, dy in ((0.0, 0.0), (radius, 0.0), (-radius, 0.0), (0.0, radius), (0.0, -radius))]
    if max(heights) - min(heights) > MAX_LANDING_PATCH_HEIGHT_CHANGE_M:
        raise ValueError('Destination terrain is too steep for mapped GPS landing')


def check_corridor(physics, start, end, cruise_z):
    length = dist(start, end)
    if length <= 0:
        raise ValueError('Route endpoints must differ')
    normal = (-(end[1] - start[1]) / length, (end[0] - start[0]) / length)
    count = ceil(length / CORRIDOR_SAMPLE_SPACING_M)
    across = ceil(2 * CORRIDOR_HALF_WIDTH_M / CORRIDOR_SAMPLE_SPACING_M)
    highest = float('-inf')
    for i in range(count + 1):
        for j in range(across + 1):
            offset = -CORRIDOR_HALF_WIDTH_M + 2 * CORRIDOR_HALF_WIDTH_M * j / across
            x, y = (start[k] + (end[k] - start[k]) * i / count + normal[k] * offset for k in range(2))
            highest = max(highest, surface_height(physics, x, y))
    clearance = cruise_z - highest
    if clearance < MIN_CLEARANCE_M:
        raise ValueError(f'Unsafe terrain corridor: clearance {clearance:.2f} m < {MIN_CLEARANCE_M} m')
    return clearance


def run_demo(*, origin, start_gps, end_gps, terrain_path, sitl_binary, on_observation=None):
    frame = GeographicFrame(*origin)
    start, end = (frame.local(*point, origin[2])[:2] for point in (start_gps, end_gps))
    if any(abs(v) > MAX_ROUTE_RADIUS_M for point in (start, end) for v in point):
        raise ValueError('Route endpoints must lie inside the terrain with a 10 m boundary margin')
    terrain = TerrainSettings(heightmap_path=str(terrain_path), world_size_m=TERRAIN_WORLD_SIZE_M,
                              samples_x=TERRAIN_SAMPLES, samples_z=TERRAIN_SAMPLES, origin_m=TERRAIN_ORIGIN_M)
    gps = GpsSettings(True, *origin)
    config = SimulationConfig(SimulationSettings(TIMESTEP_S, ceil(MISSION_TIMEOUT_S / TIMESTEP_S),
                                                 GRAVITY_VECTOR_M_S2, gui=False),
                              DroneSettings(MASS_KG, (*start, RAY_ORIGIN_Z_M)),
                              control=ControlSettings('betaflight'), sensors=SensorsSettings(gps=gps), terrain=terrain)
    vehicle = load_racer_parameters()
    probe = PyBulletPhysics(config, vehicle)
    try:
        probe.reset()
        start_ground = surface_height(probe, *start)
        end_ground = surface_height(probe, *end)
        check_landing_patch(probe, end)
        cruise_z = start_ground + TAKEOFF_HEIGHT_M
        clearance = check_corridor(probe, start, end, cruise_z)
    finally:
        probe.close()
    logger.info('Route {:.1f} m, cruise Z {:.2f} m, corridor clearance {:.2f} m', dist(start, end), cruise_z, clearance)
    config = replace(config, drone=replace(config.drone, initial_position_m=(*start, start_ground + VEHICLE_GROUND_OFFSET_M)))
    mission = Mission(start, end, cruise_z, end_ground + VEHICLE_GROUND_OFFSET_M,
                      MASS_KG, GRAVITY_M_S2, len(vehicle.rotor_positions_m) * vehicle.max_thrust_per_rotor_n)
    with ExitStack() as resources:
        physics = PyBulletPhysics(config, vehicle)
        resources.callback(physics.close)
        environment = DroneEnvironment(physics, imu=ImuSensor(GRAVITY_VECTOR_M_S2, config.sensors.imu,
                                                              sample_period_s=TIMESTEP_S),
                                       barometer=BarometerSensor(config.sensors.barometer), gps=GpsSensor(gps))
        resources.callback(environment.close)
        backend = BetaflightBackend(vehicle, UdpSitlBridge(), encoder=partial(pack_fdm, gps_enabled=True))
        resources.callback(backend.close)
        sitl = resources.enter_context(SitlSession(sitl_binary))
        observation = environment.reset()
        backend.reset(observation)
        # Register cleanup before connecting so a failed connect cannot leak the socket.
        telemetry = Telemetry(frame)
        resources.callback(telemetry.close)
        telemetry.open()
        started = time.monotonic()
        next_status = next_report = next_fdm = started
        status = None
        ready = False
        max_cross_track = 0.0
        min_clearance = float('inf')
        step = 0
        while step * TIMESTEP_S < MISSION_TIMEOUT_S:
            tick = time.monotonic()
            sitl.check_running()
            telemetry.poll()
            if tick >= next_status:
                status = sitl.commands.read_status()
                next_status = time.monotonic() + STATUS_PERIOD_S
            if not 0 <= time.monotonic() - status.received_at <= STATUS_TIMEOUT_S:
                raise RuntimeError('Stale MSP arm status')
            try:
                feedback = telemetry.feedback(time.monotonic())
            except RuntimeError:
                if ready or time.monotonic() - started > STARTUP_TIMEOUT_S:
                    raise
            else:
                ready = True
                channels = mission.command(feedback, status, step * TIMESTEP_S, TIMESTEP_S)
                sitl.client.rc.update(channels)
                if tick >= next_report:
                    logger.info('{} GPS lat={:.7f} deg lon={:.7f} deg alt_msl={:.2f} m distance_to_target={:.2f} m speed={:.2f} m/s blockers={}',
                                mission.phase, *feedback.geographic_position,
                                dist(feedback.position_m[:2], end), hypot(*feedback.velocity_m_s),
                                status.arming_disabled_flags)
                    next_report = tick + REPORT_PERIOD_S
            if mission.phase == 'complete':
                truth = observation.ground_truth
                return DemoResult(observation, tuple(mission.transitions), max_cross_track, min_clearance,
                                  dist(truth.position_m[:2], end), physics.terrain_contact(), status.armed)
            # SITL derives its clock rate from packet spacing, never burst after a slow status read.
            remaining = next_fdm - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            rotor_command = backend.command(observation)
            next_fdm = time.monotonic() + TIMESTEP_S
            observation = (environment.hold() if mission.phase in ('prearm', 'arming')
                           else environment.step(rotor_command))
            # Independent validation only, never fed back to the mission.
            truth = observation.ground_truth
            if mission.phase == 'cruise':
                dx, dy = end[0] - start[0], end[1] - start[1]
                max_cross_track = max(max_cross_track, abs(dx * (truth.position_m[1] - start[1])
                                                         - dy * (truth.position_m[0] - start[0])) / dist(start, end))
                min_clearance = min(min_clearance, truth.position_m[2] - surface_height(physics, *truth.position_m[:2]))
            if on_observation is not None:
                on_observation(observation)
            step += 1
        raise TimeoutError(f'Mission timed out in {mission.phase}')
