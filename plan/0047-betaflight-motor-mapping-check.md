# Betaflight Motor Mapping Check

## Goal

Provide a deterministic headless check that proves the configured Betaflight
motor packet ordering drives the intended racer rotor positions and roll/pitch
torque signs in PyBullet.

## Implementation

1. Add a runnable tool that injects one normalized Betaflight motor output at a
   time into the existing backend adapter and steps headless PyBullet physics.
2. Compare the resolved racer propeller index and measured roll/pitch angular
   velocity signs to the QUADX layout contract.
3. Print a per-motor pass/fail table and return a nonzero exit code on a
   mismatch.
4. Add a focused regression test for the static Betaflight-to-racer mapping.

## Interface changes

- Adds `tools/check_betaflight_motor_mapping.py`; no flight-control, YAML, or
  wire-protocol behavior changes.

## Verification

- Passed: `uv run --locked python tools/check_betaflight_motor_mapping.py`.
  All four Betaflight motor outputs produced the expected headless PyBullet
  roll/pitch torque signs.
- Passed: `uv run --locked pytest -q tests/test_betaflight_backend.py` (7 passed).
- The full suite is currently blocked by the pre-existing
  `test_viewer_task_uses_profile_launcher` expectation for a removed VS Code
  task label (280 tests passed before that failure).

## Scope and defaults

- Uses headless PyBullet and the existing `RACER_TO_BETAFLIGHT` adapter.
- Expected QUADX order is M1 rear-right, M2 front-right, M3 rear-left, and M4
  front-left.
- This checks the software packet-to-physics boundary, not real-hardware ESC
  wiring or PID tuning.

## Status

Pending

## Implementation commit

Pending
