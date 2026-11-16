# Complete simulation feedback dataclasses

## Goal
Replace flight_feedback.json with frozen validated dataclasses containing ground truth, all configured sensor readings and the attitude estimate. Export dictionaries for non-Python consumers.

## Steps and interfaces
- Add telemetry Header(frame_id, time_stamp) with simulation sec/nanosec timestamps and nested measurement headers.
- Carry drone_id, ground_truth, sensors and nullable attitude_estimate. Remove schema_version, source, armed, mode, last_applied_sequence, owner_id, state, reason and takeoff_height_m.
- Offer every observation to bounded latest-value telemetry publication on flight/drone_1/feedback independently of camera capture. Keep serialization in workers.
- Remove the JSON Schema registration and migrate tests and documentation.

## Defaults and scope
World-Z altitude is position_m[2]. IMU/rangefinder use drone body frames, GPS uses earth, and world poses use world. Disabled sensors remain null. No episode identity is retained, so consumers use backwards time to detect resets. Camera headers and image source contracts remain unchanged.

## Verification
Tests and builds will not be run by user request. Update existing protocol checks for manual execution.

## Status
Implemented; manual verification pending, uncommitted.

## Implementation commit
Pending.
