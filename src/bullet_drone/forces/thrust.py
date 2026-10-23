"""Ideal rotor thrust along body positive Z: F = k_f * omega squared."""

from bullet_drone.models import RotorSpeeds, Vector3


class RotorThrustModel:
    def __init__(self, coefficient: float) -> None:
        self.coefficient = coefficient

    def calculate(self, speeds_rad_s: RotorSpeeds) -> tuple[Vector3, Vector3, Vector3, Vector3]:
        return tuple((0.0, 0.0, self.coefficient * speed ** 2) for speed in speeds_rad_s)
