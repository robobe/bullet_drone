"""Collective-preserving allocation with bounded, uniformly scaled torques."""

from math import isfinite, sqrt

from bullet_drone.commands import InvalidCommandError, MotorAllocation, WrenchCommand
from bullet_drone.config import ConfigurationError
from bullet_drone.vehicle import VehicleParameters


def _inverse(matrix: tuple[tuple[float, ...], ...]) -> tuple[tuple[float, ...], ...]:
    """Small 4x4 Gauss-Jordan inverse; no numerical-library dependency needed."""
    rows = [list(row) + [float(i == j) for j in range(4)] for i, row in enumerate(matrix)]
    for column in range(4):
        pivot = max(range(column, 4), key=lambda i: abs(rows[i][column]))
        if abs(rows[pivot][column]) < 1e-12:
            raise ConfigurationError("Rotor geometry cannot allocate independent thrust and three torques")
        rows[column], rows[pivot] = rows[pivot], rows[column]
        divisor = rows[column][column]
        rows[column] = [value / divisor for value in rows[column]]
        for i in range(4):
            if i != column:
                multiplier = rows[i][column]
                rows[i] = [a - multiplier * b for a, b in zip(rows[i], rows[column])]
    return tuple(tuple(row[4:]) for row in rows)


class MotorMixer:
    def __init__(self, vehicle: VehicleParameters) -> None:
        self.vehicle = vehicle
        yaw_per_thrust = vehicle.torque_coefficient / vehicle.thrust_coefficient
        self._matrix = (
            (1.0, 1.0, 1.0, 1.0),
            tuple(r[1] for r in vehicle.rotor_positions_m),
            tuple(-r[0] for r in vehicle.rotor_positions_m),
            tuple(sign * yaw_per_thrust for sign in vehicle.reaction_torque_signs),
        )
        self._inverse = _inverse(self._matrix)

    def mix(self, request: WrenchCommand) -> MotorAllocation:
        if not isinstance(request, WrenchCommand):
            raise InvalidCommandError("Mixer requires a WrenchCommand")
        limit = self.vehicle.max_thrust_per_rotor_n
        collective = min(request.thrust_n / 4, limit)
        # Zero collective contribution: these corrections sum to zero thrust.
        deltas = tuple(sum(row[j + 1] * request.torque_nm[j] for j in range(3)) for row in self._inverse)
        if not all(isfinite(delta) for delta in deltas):
            raise InvalidCommandError("Torque request is too large to allocate")
        scale = 1.0
        for delta in deltas:
            if delta > 0:
                scale = min(scale, (limit - collective) / delta)
            elif delta < 0:
                scale = min(scale, collective / -delta)
        thrusts = tuple(min(limit, max(0.0, collective + scale * delta)) for delta in deltas)
        achieved = tuple(sum(a * b for a, b in zip(row, thrusts)) for row in self._matrix)
        speeds = tuple(min(self.vehicle.max_speed_rad_s, sqrt(force / self.vehicle.thrust_coefficient)) for force in thrusts)
        return MotorAllocation(
            requested=request, achieved=WrenchCommand(achieved[0], achieved[1:]),
            speeds_rad_s=speeds, rotor_thrusts_n=thrusts,
            saturated=request.thrust_n > 4 * limit or scale < 1.0, torque_scale=scale,
        )
