# Altitude-hold sensor configuration

## Goal

Enable the rangefinder and IMU-based attitude estimate in altitude hold so those observation fields contain readings.

## Implementation steps

1. Configure the rangefinder as enabled in the example's `SimulationConfig`.
2. Select the IMU attitude source so `RcSimulation` constructs and updates the attitude estimator.
3. Record the configuration change in the plan index.

## Interface changes

None. Use existing `RangefinderSettings`, `SensorsSettings`, and `PoseSources` options.

## Verification

- Confirm from configuration assembly that rangefinder readings are enabled and the IMU attitude estimator is selected.
- `git diff --check`.

## Scope and defaults

- Keep the mission controller on ground-truth feedback; these readings are additional observation outputs only.
- Use the existing rangefinder range/mount and attitude-estimator tuning defaults.

## Status

Implemented and verified; uncommitted.

- Altitude hold enables `RangefinderSettings` and sets `PoseSources(attitude='imu')`.
- The altitude-hold usage guide now describes these readings as enabled.
- The headless mission completed; its 1 s observation showed `distance_cm=37.56`, `valid=True`, and a populated attitude estimate.
- `git diff --check` passed.

## Implementation commit

Pending
