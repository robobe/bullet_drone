# 0037: Sensor pose sources

## Goal

Opt in through physics YAML to barometer render altitude and IMU-derived attitude
for both rendering and app feedback. Noise belongs to the existing physics sensors.

## Implementation

1. Add validated pose-source and complementary-filter settings to SimulationConfig
   and YAML loading, retaining ground-truth defaults.
2. Add a timestamped attitude estimate to observations, separate from raw readings
   and ground truth. Run a quaternion Mahony complementary filter at each physics
   step with accelerometer rejection, gyro bias estimation and reset lifecycle.
3. Seed once from the known spawn orientation. Use only IMU readings thereafter.
4. Resolve output pose centrally, preserving true XY/velocity and source stamps.
   Apply the same estimate to camera and app feedback, including stopped poses.
5. Add external-RC --config using the existing YAML loader. Keep old launch defaults.
6. Accept imu_complementary in the shared flight contract/schema and verify app use.
7. Document selectors, noise, defaults and limitations, and run focused tests.

## Interfaces and defaults

- pose_sources.altitude: ground_truth (default) or barometer.
- pose_sources.attitude: ground_truth (default) or imu.
- attitude_estimator.kp_s_inv: 2.0, nonnegative.
- attitude_estimator.ki_s_inv2: 0.05, nonnegative.
- attitude_estimator.acceleration_rejection_fraction: 0.2, strictly between 0 and 1.
- Optional DroneObservation.attitude_estimate, None unless estimation is selected.
- Existing sensor standard deviations, biases and seeds remain authoritative.
- Disabled required sensors cause configuration failure, without truth fallback.
- App altitude remains the existing barometer measurement. Internal stabilization
  remains ideal. Legacy standalone render profiles are unchanged.

## Verification

Filter math, frames, noise propagation/repeatability, reset/failure lifecycle,
configuration validation, unchanged commanded physics, publication selection,
stopped caching, spawn replacement, shared schema and app acceptance. Results:

- Physics suite: 632 passed, 2 unrelated PlotJuggler task tests failed because
  the earlier single-workspace-task change left folder tasks empty. Those task
  files and obsolete expectations were not changed by this implementation.
- Shared messages: 129 passed. App: 67 passed.
- Native Godot TCP/SHM adapter smoke: 2 passed.
- Real NVIDIA Godot scene smoke passed RGB orientation, immediate pose/readback,
  FOV, UI exclusion and truck geometry checks.
- CLI help exposes --config. YAML remains opt-in with zero default noise.

No full noisy closed-loop flight or hardware calibration was performed.

## Limits

No magnetometer or position estimator. Yaw drifts. Acceleration can corrupt tilt
and the norm gate cannot detect every acceleration. Rendered noisy poses affect
image-based collision reports. No calibrated hardware behavior is claimed.

## Status

Implemented; verification results and existing failures recorded above.
Implementation commit: `7c6e86617a25fc300adfaa1a3116c225750f78fc`.
