"""Body reaction -Omega cross H in N*m, with physical rotor spin around body +Z."""

from bullet_drone.models import RotorSpeeds, Vector3


def calculate(body_rates_rad_s: Vector3, speeds_rad_s: RotorSpeeds,
              physical_spin_signs: tuple[int, int, int, int], rotor_inertia_kg_m2: float) -> Vector3:
    momentum = sum(sign * rotor_inertia_kg_m2 * speed for sign, speed in zip(physical_spin_signs, speeds_rad_s))
    roll, pitch, _ = body_rates_rad_s
    return (-pitch * momentum, roll * momentum, 0.0)
