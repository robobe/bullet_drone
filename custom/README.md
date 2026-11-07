# PX4 Iris custom URDF

`iris.yaml` is a headless physics profile that selects `iris.urdf` by a path
relative to this directory. Load it with `YamlConfigLoader` or pass it to the
YAML-driven external-RC example.

## Source and conversion

This is a compact PyBullet-compatible port of the quadrotor geometry and
actuator constants in the [PX4 SITL Gazebo Iris model](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/models/iris/iris.sdf.jinja).
The PX4 [motor plugin](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/src/gazebo_motor_model.cpp)
uses thrust proportional to squared angular speed and applies reaction torque
using a moment-to-thrust ratio.

The model uses the published 1.5 kg base mass, body inertia and dimensions,
rotor locations, 0.128 m rotor collision radius, motor constant
`5.84e-6 N/(rad/s)^2`, moment constant `0.06 m`, and maximum speed `1100 rad/s`.
The loader's URDF coefficients use RPM-based constants, so the file stores:

```text
rpm_to_rad_s = 60 / (2*pi)
kf_urdf = 5.84e-6 / rpm_to_rad_s^2 = 6.4042766336e-8
km_urdf = (5.84e-6 * 0.06) / rpm_to_rad_s^2 = 3.84256598016e-9
max_thrust_per_rotor = 5.84e-6 * 1100^2 = 7.0664 N
thrust2weight = 4 * 7.0664 / (1.5 * 9.81) = 1.92086986069
```

PX4's original rotor sequence is reordered to the project's fixed `(+, -, +, -)`
reaction-sign convention: custom prop0/1/2/3 map to PX4 rotor0/2/1/3. Rotor
links are massless because this simulation models their forces at the vehicle
level rather than simulating spinning rotor bodies. The profile is a testable
simulation reference, not a calibrated real-aircraft model.
