# Simulation usage guide

## Goal

Add a beginner-friendly guide for running the existing altitude-hold example and reading simulation and sensor data at each physics step.

## Implementation steps

1. Create `docs/usage/README.md` with the run command, the observation flow, and a Python callback example that prints ground truth, IMU, and barometer values.
2. Explain the observation fields, optional sensor readings, standard output versus Loguru diagnostics, and link to the sensor design reference.
3. Add this plan to `plan/README.md`.

## Interface changes

None. The guide uses `examples.altitude_hold.demo.run_demo(on_observation=...)` and the existing `DroneObservation` API.

## Verification

- Confirm the documented CLI command runs headless.
- Run the documented callback snippet and confirm it prints the reset observation and subsequent per-step observations with sensor values.

## Scope and defaults

- Documentation only; do not change simulation behavior or add dependencies.
- Use the current altitude-hold example and its callback API.

## Status

Implemented and verified; uncommitted.

- Headless altitude-hold CLI completed successfully.
- The documented callback printed 7,879 observations, from reset at step 0 through step 7,878, including ground truth, IMU, and barometer data.
- `git diff --check` passed.

## Implementation commit

Pending
