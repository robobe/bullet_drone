"""Rotor drag reaction on the body, using explicitly declared yaw signs."""

from bullet_drone.models import RotorSpeeds, Vector3


class ReactionTorqueModel:
    def __init__(self, coefficient: float, signs: tuple[int, int, int, int]) -> None:
        self.coefficient = coefficient
        self.signs = signs

    def calculate(self, speeds_rad_s: RotorSpeeds) -> Vector3:
        yaw = sum(sign * self.coefficient * speed ** 2 for sign, speed in zip(self.signs, speeds_rad_s))
        return (0.0, 0.0, yaw)
