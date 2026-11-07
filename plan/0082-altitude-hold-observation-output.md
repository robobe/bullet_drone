# Altitude-hold observation output

## Goal

Print readable altitude-hold ground-truth and sensor observations to stdout once per simulated second, with colored mission-state changes.

## Implementation steps

1. Add example-owned formatting for observation snapshots and colored phase changes.
2. Emit status snapshots at one-second simulation-time intervals from the altitude-hold loop.
3. Verify the cadence and reported state/sensor fields with a focused regression.

## Interface changes

No library API changes. `RcSimulation` and the existing observation callback are unchanged.

## Verification

- `uv run python -m examples.altitude_hold.main --headless` completed the mission and printed ground truth and sensor values once per simulated second.
- `uv run pytest -q tests/test_altitude_hold_example.py` — 16 passed.
- `git diff --check` passed.

## Scope and defaults

- Output is owned by the altitude-hold example and written to stdout.
- The one-second interval is measured in simulation time; phase changes are printed immediately.
- Sensor slots that are unavailable are printed as `None`.

## Status

Implemented and verified; uncommitted.

## Implementation commit

Pending
