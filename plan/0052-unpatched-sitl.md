# Unpatched Betaflight SITL integration

Status: Implemented and verified with documented test limitations. Firmware removal changes are uncommitted.
Implementation commits: `3cb005baf45d364be6f27cc7e7c5b96753cde582`; removal implementation commit Pending.

Goal (revised): Replace the patched workflow with upstream Betaflight 2026.6.2 SITL. Map simulated IMU, barometer-pressure-derived altitude and pose into the existing FDM packet, configure MSP input/feedback with GPS and magnetometer use disabled, and verify the sensor and motor loop. MAVLink work is deferred.

Steps: retain the upstream-compatible FDM encoder, remove the firmware patch/build flag, build clean pinned upstream firmware, configure receiver and sensors, verify pressure conversion, then run existing regressions and live checks. Document configuration and limitations. The authorized removal and verification are tracked in [plan 0054](0054-remove-sitl-patch.md).

Interfaces: preserve the existing FDM/motor packet layouts and backend API. Use only unpatched SITL; no patched encoder or extra selection option.

Verification: packet field/pressure tests, setup checks, upstream SITL sensor and motor exchange, MSP feedback, arming/failsafe and headless hover. No hardware flight or MAVLink implementation.

Outcome: Initial FDM mapping committed. The earlier requirement to preserve the patch and add backend selection was superseded by the user's request to remove the patch. See plan 0054 for the implementation and actual verification results.

Plan 0054 completed the clean upstream build, profile changes and live validation:
pressure-derived altitude, motor exchange, arming, RC-loss disarming and motor
timeout passed. The canonical 20-second hover check passed with 0.28 m peak
altitude error. The full Python suite had 705 passes and three failures also
reproduced on untouched HEAD. A separate noncanonical-profile hover run exposed
the existing validator's post-spool altitude reference limitation; no controller
tuning or validator thresholds were changed. MAVLink remains deferred.
