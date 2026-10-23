# Shared frame math

`bullet_drone.geometry` owns quaternion algebra and coordinate transformations
used by IMU readings, ground-truth feedback and flight controllers. Its direct
project dependency is the tuple types in `models`; it imports no controller,
command, configuration, physics or rendering implementation.

## Conventions and interface

World coordinates are right-handed, with X/Y horizontal and Z up. Body
coordinates are right-handed, with X forward, Y left and Z up. Orientation
quaternions use XYZW order and rotate body coordinates into world coordinates.
Angles use radians and ZYX Euler composition (yaw, pitch, roll).

- `normalize(q)` returns a unit quaternion; orientation norm must be finite and nonzero.
- `conjugate(q)` and `multiply(a, b)` perform raw quaternion algebra. Multiplication
  order matters; neither function normalizes its operands or result. A quaternion
  with a zero scalar part may represent a physical vector, whose magnitude must
  be preserved.
- `from_rpy(rpy)` converts roll/pitch/yaw radians to a body-to-world quaternion.
- `world_to_body(vector, orientation)` applies inverse orientation to any world
  XYZ vector. It preserves the vector's physical units, including m/s² for
  specific force and rad/s for angular velocity. Vector diagnostics use `vector`,
  not an angular-velocity label.
- `body_to_world(vector, orientation)` applies the forward orientation to a body
  vector, preserving its units; aerodynamic rotor positions and axes use it.
- `attitude_error(current, target)` returns the shortest rotation vector in the
  current body frame. Quaternion sign equivalence, the deterministic exact
  half-turn choice and the existing small-angle threshold are preserved.
- `yaw_from_quaternion(orientation)` returns world ZYX heading in radians.

Inputs accept finite numeric tuples/lists of the required length, excluding
booleans. Invalid values raise `GeometryError`, a domain-neutral `ValueError`.
Validation belongs to this module; callers do not import command-private helpers
for frame math. There is no adapter hierarchy, mutable state or implicit sensor
estimation.

## Errors at each seam

IMU readings and `IdealStateFeedback.read()` consume ground-truth source state.
They let `GeometryError` identify invalid source geometry. Environment sampling
failure still invalidates the episode and requires reset; it does not become a
retryable command rejection merely because the cause is geometry.

`AttitudeController.rates()` and `InternalFlightController.update()` preserve
command-facing `InvalidCommandError` behavior by translating geometry errors,
retaining the original error as the cause. Feedback validation and attitude
calculation complete before PID history or mode changes. Control gains, limits,
rate control, RC translation, mixing and physics behavior are unchanged.

## Migration and verification

Replace imports of `bullet_drone.controllers.geometry` with
`bullet_drone.geometry`. The former module is removed without forwarding aliases.
Source-state consumers that caught `InvalidCommandError` for invalid geometry
should catch `GeometryError` (or `ValueError`) instead. Controller-facing methods
continue to raise `InvalidCommandError`.

`tests/test_geometry.py` exercises rotations, raw algebra, normalization, shortest
rotation and quaternion sign behavior, malformed input, neutral diagnostics,
controller rejection without PID changes, environment reset after invalid source
geometry, and sensor imports without controllers. Geometry-only tests previously
in the controller test file now live at the shared math interface. Full flight
checks continue exercising the same control and physics chain in Angle and Acro.
