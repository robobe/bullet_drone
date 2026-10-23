# 0014: Shared frame math

## Goal

Give quaternion/frame transformations neutral ownership and diagnostics so IMU readings do not depend on controller or command internals.

## Implementation steps

- Move geometry operations to bullet_drone.geometry and delete controllers/geometry.py; update all repository callers, examples and tests without a compatibility forwarding module.
- Keep XYZW body-to-world quaternions, right-handed world/body axes, ZYX angles, multiplication order, shortest-rotation/sign handling and numerical thresholds unchanged.
- Own finite vector/quaternion validation in the math module, with GeometryError(ValueError) and neutral field labels. Retain raw quaternion multiplication without implicit normalization.
- Keep attitude/rate control policy in controllers. Translate GeometryError to InvalidCommandError in AttitudeController.rates and InternalFlightController.update before PID state changes; ground-truth feedback and IMU errors remain neutral.
- Add focused math tests and sensor/controller regression checks; update frame documentation, sensor design and domain glossary.

## Interface changes

Geometry functions move from bullet_drone.controllers.geometry to bullet_drone.geometry. GeometryError replaces command errors at the math/source-state interface. Controller-facing exception behavior remains InvalidCommandError. No sensor/configuration/schema changes.

## Verification

Test rotations, multiplication, normalization, sign equivalence, shortest/180-degree rotations, malformed values and diagnostic labels. Verify source errors invalidate the environment, controller rejection preserves PID history, and importing sensors no longer imports controllers. Verification completed:

- Focused geometry/controller/sensor checks: 85 passed.
- `uv run --locked pytest -q`: 333 passed, 1 skipped (optional MCAP dependency unavailable).
- Headless altitude hold completed at 23.225 simulated seconds.
- Headless Angle square completed at 44.496 simulated seconds.
- Headless Acro square completed at 44.500 simulated seconds.
- Import regression verifies sensors do not load controllers; no old geometry imports remain in source/examples/tests.
- `git diff --check`: passed. GUI not exercised.

## Scope and defaults

Remove the old import path as recommended and accepted. No YAML/settings refactor, sensor hierarchy, adapter framework, new dependency, estimator or shared runner. Use named module-top constants for numerical tolerances.

## Status

Complete; committed.

## Implementation commit

`3f92fe2fbcde954e2ece0eee279424380325ae86`
