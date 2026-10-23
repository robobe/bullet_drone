# 0011: IMU and barometer sensors

## Goal

Return configurable IMU and barometer measurements in DroneObservation without changing ground-truth physics or controller feedback.

## Implementation steps

- Add separate ImuSensor and BarometerSensor classes, immutable readings, reset/sample lifecycle and observation identity stamps.
- Calculate body-frame specific force from consecutive world velocities minus gravity; transform world angular velocity into body axes. Reset assumes zero world acceleration; mounting is aligned at the center of mass.
- Model constant-temperature atmospheric pressure and derive world-Z altitude from noisy pressure. Defaults: 101325 Pa, 288.15 K, reference Z 0 m, Earth atmospheric gravity and dry-air gas constant.
- Add Python/YAML settings: enabled by default, zero-noise defaults, per-axis accelerometer/gyro Gaussian standard deviations and fixed biases, scalar pressure noise/bias, independent seeded streams repeating on reset.
- Inject sensors into DroneEnvironment; RcSimulation constructs them from settings. Sample after reset and every successful physics step. Direct environment callers explicitly inject sensors.
- Validate parameters, sample chronology and pressure; sensor failures require reset. Update MCAP schemas, design menu, sensor documentation and configuration examples.

## Interface changes

Typed ImuReading and BarometerReading in SensorReadings; sensor constructors and reset(state)/sample(state); SimulationConfig.sensors; optional IMU/barometer injection into DroneEnvironment. Existing MCAP sensor objects become documented typed payloads.

## Verification

Unit checks for IMU stationary/free-fall/acceleration/frames, barometric round-trips, bias/noise repeatability, invalid settings and chronology. Integration checks for observation identities, lifecycle, MCAP payloads and unchanged physics/control.

Verification completed:

- `uv run --locked pytest -q`: 313 passed, 1 skipped (optional `mcap_bug` file-readback module unavailable).
- `uv run --locked pytest -q tests/test_sensors.py`: 38 passed.
- Altitude hold headless: completed at 23.225 simulated seconds.
- Angle position square headless: completed at 44.496 simulated seconds.
- Acro position square headless: completed at 44.500 simulated seconds.
- Real recorder-adapter payload/schema checks pass without the optional MCAP service; GUI was not exercised.
- `git diff --check`: passed.

## Scope and defaults

No estimator, sensor-driven feedback, random-walk bias, sampling scheduler or new flight example. Both sensors enabled with zero noise in RcSimulation; rangefinder remains unavailable. No new dependencies. Named physical constants at module tops.

## Status

Complete; committed.

## Implementation commit

`84449a54c030f9393b41b8048bf31e9cd8caa381`
