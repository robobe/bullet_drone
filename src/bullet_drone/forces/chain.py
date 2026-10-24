"""Compose aerodynamic calculations; PyBullet alone applies and integrates forces."""

from bullet_drone.config import ForcesSettings, WindSettings
from bullet_drone.contracts import SurfaceQueries
from bullet_drone.forces import angular_damping, body_drag, flapping, ground_effect, gyroscopic_torque, inflow, propwash, rotor_drag
from bullet_drone.forces._math import BODY_UP, ZERO_VECTOR, add, cross, scale
from bullet_drone.forces.wind import WindModel
from bullet_drone.geometry import body_to_world, world_to_body
from bullet_drone.models import DroneState, ForceOutput, PropellerOutput, RotorSpeeds
from bullet_drone.vehicle import VehicleParameters


class ForceChain:
    def __init__(self, settings: ForcesSettings, wind: WindSettings, vehicle: VehicleParameters) -> None:
        self._settings = settings
        self._wind = WindModel(wind)
        self._vehicle = vehicle
        self._active = any(getattr(settings, name).enabled for name in
                           ("body_drag", "rotor_drag", "angular_damping", "inflow", "flapping",
                            "gyroscopic", "ground_effect", "propwash"))

    def reset(self) -> None:
        self._wind.reset()

    @property
    def wind(self):
        return self._wind.snapshot

    def set_wind(self, settings: WindSettings, simulation_time_s: float) -> None:
        self._wind.set_settings(settings, simulation_time_s)

    def calculate(self, state: DroneState, speeds_rad_s: RotorSpeeds, baseline: PropellerOutput,
                  surfaces: SurfaceQueries) -> ForceOutput:
        ambient = self._wind.velocity(state.simulation_time_s)
        if not self._active:
            return ForceOutput(baseline.forces_n, ZERO_VECTOR, baseline.reaction_torque_nm)
        settings = self._settings
        density, radius = settings.air_density_kg_m3, settings.rotor_radius_m
        orientation = state.orientation_xyzw
        axis = body_to_world(BODY_UP, orientation)
        positions = tuple(add(state.position_m, body_to_world(offset, orientation))
                          for offset in self._vehicle.rotor_positions_m)
        base_thrusts = tuple(max(0.0, force[2]) if speed > 0 else 0.0
                             for force, speed in zip(baseline.forces_n, speeds_rad_s))
        rates = world_to_body(state.angular_velocity_rad_s, orientation)

        def airflow(point, *, exclude_rotor=None):
            wake = (propwash.velocity_at(point, positions, axis, base_thrusts, density, radius,
                                        settings.propwash, exclude_rotor=exclude_rotor)
                    if settings.propwash.enabled else ZERO_VECTOR)
            return world_to_body(add(state.linear_velocity_m_s, scale(add(ambient, wake), -1)), orientation)

        sample = add(state.position_m, body_to_world(settings.propwash.body_sample_offset_m, orientation))
        body_air = airflow(sample)
        body_force = ZERO_VECTOR
        if settings.body_drag.enabled:
            body_force = add(body_force, body_drag.calculate(body_air, density, settings.body_drag.cd_area_m2))
        if settings.rotor_drag.enabled:
            body_force = add(body_force, rotor_drag.calculate(body_air, speeds_rad_s,
                                                             settings.rotor_drag.coefficients_n_per_rad_s_m_s))
        torque = baseline.reaction_torque_nm
        if settings.angular_damping.enabled:
            torque = add(torque, angular_damping.calculate(rates, settings.angular_damping.coefficients_nm_per_rad_s))
        if settings.gyroscopic.enabled:
            spins = tuple(-sign for sign in self._vehicle.reaction_torque_signs)
            torque = add(torque, gyroscopic_torque.calculate(rates, speeds_rad_s, spins,
                                                             settings.gyroscopic.rotor_inertia_kg_m2))
        forces = []
        for index, (force, speed, offset, position) in enumerate(zip(
                baseline.forces_n, speeds_rad_s, self._vehicle.rotor_positions_m, positions)):
            local_air = add(airflow(position, exclude_rotor=index), cross(rates, offset))
            multiplier = 1.0
            if settings.inflow.enabled:
                multiplier *= inflow.multiplier(local_air, speed, radius, settings.inflow.coefficient,
                                                 settings.inflow.minimum_multiplier)
            if settings.ground_effect.enabled:
                hit = surfaces.surface_below(position, settings.ground_effect.query_range_m)
                multiplier *= ground_effect.multiplier(hit, axis, radius, settings.ground_effect)
            # Baseline custom in-plane forces are preserved; only axial thrust is modified.
            modified = (force[0], force[1], force[2] * multiplier)
            if settings.flapping.enabled:
                modified = add(modified, flapping.calculate(local_air, modified[2], speed, radius,
                                                            settings.flapping.coefficient))
            forces.append(modified)
        return ForceOutput(tuple(forces), body_force, torque)
