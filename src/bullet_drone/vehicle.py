"""Vehicle geometry and ideal actuator constants, independent of PyBullet."""

from dataclasses import dataclass
from importlib.resources import files
from math import isfinite, pi, sqrt
from xml.etree import ElementTree as ET

from bullet_drone.config import ConfigurationError
from bullet_drone.models import Vector3


@dataclass(frozen=True, slots=True)
class VehicleParameters:
    rotor_positions_m: tuple[Vector3, Vector3, Vector3, Vector3]
    reaction_torque_signs: tuple[int, int, int, int]
    thrust_coefficient: float  # N / (rad/s)^2
    torque_coefficient: float  # N*m / (rad/s)^2
    max_thrust_per_rotor_n: float

    def __post_init__(self) -> None:
        if len(self.rotor_positions_m) != 4 or any(len(r) != 3 for r in self.rotor_positions_m):
            raise ConfigurationError("Vehicle requires four 3D rotor positions")
        positions = tuple(tuple(float(v) for v in r) for r in self.rotor_positions_m)
        if any(not isfinite(v) for r in positions for v in r):
            raise ConfigurationError("Rotor positions must be finite")
        signs = tuple(self.reaction_torque_signs)
        if len(signs) != 4 or any(type(sign) is not int or sign not in (-1, 1) for sign in signs):
            raise ConfigurationError("Reaction torque signs must be four values of +1 or -1")
        # Equal collective assumes a balanced layout with identical actuators.
        if any(abs(sum(r[axis] for r in positions)) > 1e-12 for axis in (0, 1)) or sum(signs) != 0:
            raise ConfigurationError("Mixer requires a balanced four-rotor layout")
        for name in ("thrust_coefficient", "torque_coefficient", "max_thrust_per_rotor_n"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value) or value <= 0:
                raise ConfigurationError(f"{name} must be positive and finite")
        object.__setattr__(self, "rotor_positions_m", positions)
        object.__setattr__(self, "reaction_torque_signs", signs)

    @property
    def max_speed_rad_s(self) -> float:
        return sqrt(self.max_thrust_per_rotor_n / self.thrust_coefficient)


def load_racer_parameters() -> VehicleParameters:
    """Read the packaged URDF once; hardware limits do not scale with YAML mass."""
    try:
        root = ET.fromstring(files("bullet_drone").joinpath("assets/racer/racer.urdf").read_bytes())
        properties = root.find("properties")
        nominal_mass = float(root.find("link[@name='base_link']/inertial/mass").get("value"))
        positions = tuple(
            tuple(map(float, root.find(f"link[@name='prop{i}_link']/inertial/origin").get("xyz").split()))
            for i in range(4)
        )
        conversion = (60 / (2 * pi)) ** 2
        return VehicleParameters(
            rotor_positions_m=positions,
            reaction_torque_signs=(1, -1, 1, -1),
            thrust_coefficient=float(properties.get("kf")) * conversion,
            torque_coefficient=float(properties.get("km")) * conversion,
            max_thrust_per_rotor_n=nominal_mass * 9.81 * float(properties.get("thrust2weight")) / 4,
        )
    except (OSError, ET.ParseError, AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise ConfigurationError(f"Invalid bundled racer actuator parameters: {exc}") from exc
