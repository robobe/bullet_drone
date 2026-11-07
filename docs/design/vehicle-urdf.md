# Vehicle URDF selection

`drone.urdf` in physics YAML selects either the packaged `racer` or
`heavy_quad` model, or a custom URDF file. Relative custom paths are resolved
from the YAML file's directory. Absolute paths are accepted. Code that constructs
`DroneSettings` directly uses the process working directory for relative paths.

For example, with `configs/custom.yaml` and `vehicles/quad.urdf`:

```yaml
drone:
  # Built-in model name or a custom four-rotor URDF path, relative to this YAML.
  urdf: vehicles/quad.urdf
  # Positive vehicle mass in kg; scales base-link mass and inertia in PyBullet.
  mass_kg: 1.2
  # Initial world position in metres.
  initial_position_m: [0.0, 0.0, 1.0]
```

Custom URDFs must follow the current equal-actuator quadrotor contract:

- Include `base_link` with `inertial/mass`.
- Include `prop0_link` through `prop3_link`. Each needs
  `inertial/origin xyz` giving its rotor position in metres, relative to the
  vehicle center. Keep the existing rotor index order and reaction signs
  `(+, -, +, -)` used by the mixer.
- Include a root `properties` element with `kf`, `km`, and `thrust2weight`.
  The loader converts the thrust and torque coefficients from the URDF's RPM
  convention to SI units and derives per-rotor maximum thrust from the nominal
  base-link mass and thrust-to-weight ratio.
- Use a balanced four-rotor layout. The motor mixer assumes four identical
  rotors and equal collective allocation.

The configured `drone.mass_kg` scales the loaded base-link mass and inertia; it
does not replace the URDF nominal mass used to derive maximum motor thrust.
Missing files, malformed XML, or missing actuator/link properties fail during
vehicle parameter loading with `ConfigurationError`. Invalid geometry may also
be rejected by PyBullet when physics is reset.
