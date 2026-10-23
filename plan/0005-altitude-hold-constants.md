# Altitude hold named constants

## Goal
Refactor the altitude-hold example to follow the named-constants rule without
changing control behavior, timing, GUI framing or CLI exit statuses.

## Implementation steps
- Group mission targets, limits, tolerances, durations and frame values after imports.
- Name simulation defaults, physical parameters and immutable PID settings.
- Name GUI framing values and CLI exit statuses at module level.
- Document where each group of example constants is maintained.

## Interface changes
None; retain entry points, function signatures and existing defaults.

## Verification
Run existing example/controller tests, the full suite and a headless mission;
verify safe imports/help and unchanged flight timing and landing.

## Scope/defaults
Only `examples/altitude_hold` and its documentation. Keep constants in their
owning modules; no new configuration layer, dependencies or behavior changes.

## Status
Implemented and verified; uncommitted.

Verification outcome:
- `uv run --locked pytest -q`: 254 passed, 1 skipped (optional telemetry integration).
- Headless flight retained phase times: takeoff 0.500 s, hold 5.179 s, land
  15.179 s, disarmed 22.225 s, complete 23.225 s; final altitude 0.0135 m.
- Existing import/help, controller, cleanup and exact-deadline tests passed.
- Numeric-literal inspection leaves only obvious zero initialization/comparison
  and one-step arithmetic in function bodies; domain/tuning values are named.
- GUI values were preserved; interactive GUI viewing was not repeated.

Implementation commit hash: Pending (no commit requested).
