# Faster terrain following

## Goal
Increase the terrain-following example's cruise speed from 2 to 3 m/s.

## Implementation steps
1. Raise the navigation speed cap to 3 m/s and lookahead to 4 m so the position controller can request that speed.
2. Update the example guide and verify the configured controller can request 3 m/s on a straight leg.
3. Run focused tests and a complete headless SITL mission.

## Interface changes
None. Example constants and documented defaults only.

## Verification
- Focused terrain-following tests: 8 passed. Ruff and git diff --check passed.
- Complete paced headless SITL flight: 87.458 simulated seconds, home landing, disarm and owned firmware cleanup passed.
- Periodic outbound/return clearance readings ranged from 2.8 to 3.46 m; these are sampled diagnostics, not exact extrema.
- Rendered flight was not repeated.

## Scope and defaults
Keep the route, 3 m clearance target, acceleration/tilt limits, landing behavior and firmware unchanged. No new options or dependencies.

## Status
Implemented and verified; uncommitted.

## Implementation commit
Pending.
