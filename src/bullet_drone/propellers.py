"""Ideal propellers: k_f*omega² thrust and signed k_m*omega² yaw reaction."""

from bullet_drone.commands import InvalidCommandError, _values
from bullet_drone.forces.reaction_torque import ReactionTorqueModel
from bullet_drone.forces.thrust import RotorThrustModel
from bullet_drone.models import PropellerOutput, RotorSpeeds
from bullet_drone.vehicle import VehicleParameters


class IdealPropellerModel:
    def __init__(self, vehicle: VehicleParameters) -> None:
        self._thrust = RotorThrustModel(vehicle.thrust_coefficient)
        self._reaction = ReactionTorqueModel(vehicle.torque_coefficient, vehicle.reaction_torque_signs)

    def calculate(self, speeds_rad_s: RotorSpeeds) -> PropellerOutput:
        speeds = _values(speeds_rad_s, 4, "speeds_rad_s")
        if any(v < 0 for v in speeds):
            raise InvalidCommandError("Propeller speeds cannot be negative")
        return PropellerOutput(self._thrust.calculate(speeds), self._reaction.calculate(speeds))
