# Colored position-square status

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

## Goal
Make phase changes and waypoint progress easy to distinguish in the position-square terminal output.

## Implementation steps
1. Add focused diagnostic helpers using the existing Loguru color-tag pattern.
2. Log previous/new phase and active/settled waypoint status with target coordinates.
3. Enable colors for terminal output at the CLI entry point; redirected logs remain plain.
4. Document the colors and verify colored/plain output plus square mission behavior.

## Interfaces and scope
No flight-control, renderer, mission timing, telemetry, dependency, or Acro changes. No new CLI options. Logging configuration stays at the application entry point.

## Verification
- Square mission and diagnostics suite: 21 passed, including complete flight, deadline, failure cleanup, and colored/plain output checks.
- Additional CLI regression run with new assertions: passed with all five waypoints each active/settled exactly once, phase transitions present, and no ANSI escapes in redirected stderr. Together with both color-output cases: 3 passed.
- `git diff --check` passed. Full repository suite and another graphical flight were skipped for this logging-only change.
- No commit or push requested.
