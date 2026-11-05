# Unpatched Betaflight SITL integration

Status: In progress. Implementation commit: `3cb005baf45d364be6f27cc7e7c5b96753cde582`.

Goal: Support upstream Betaflight 2026.6.2 SITL while preserving the existing patched workflow. Map simulated IMU, barometer-derived altitude and pose into the FDM packet, configure MSP input and MAVLink telemetry with GPS and magnetometer disabled, and verify the full sensor, motor and telemetry loop.

Steps: add and test an unpatched FDM encoder, integrate an explicit backend selection, build upstream firmware with MAVLink, configure receiver and sensors, verify axis signs and pressure conversion, then run the existing regressions and headless hover check. Document configuration and limitations.

Interfaces: preserve the existing FDM packet layout and patched encoder; add a separate upstream-compatible encoder and explicit selection.

Verification: packet field tests, invalid sensor tests, upstream SITL sensor and motor exchange, MAVLink altitude, arming and failsafe tests, full regression suite and headless hover. No hardware flight.

Outcome: Initial FDM mapping committed. All 8 tests in `tests/test_betaflight_backend.py` pass, and `git diff --check` passes. Backend selection, preservation of the patched encoder, upstream integration, documentation and remaining verification are pending.
