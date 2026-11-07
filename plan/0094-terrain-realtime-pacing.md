# Terrain realtime pacing

## Goal
Restore near-realtime terrain flight and approximately 30 wall-clock camera FPS without FDM catch-up bursts.

## Steps
1. Measure status blocking and reproduce timing overhead.
2. Move periodic status reads onto one example-owned worker, using the latest completed status and preserving freshness/error checks.
3. Add deterministic loop regression with slow status replies and an observer stall; run focused tests and full SITL flight.

## Interface changes
None. Terrain example only; existing MSP API and camera configuration unchanged.

## Verification
Baseline: 51 status requests blocked the flight loop for 0.633 seconds during a 5 simulated-second run (5–22 ms per request).
- Regression failed before the fix (0.579 s send span for 0.5 simulated seconds plus a 40 ms stall) and passed afterward. It also verifies stale/error aborts, worker cleanup and no catch-up bursts.
- 86 focused terrain, Betaflight and rendering tests passed; 2 opt-in live tests skipped. Ruff and git diff --check passed.
- Full headless mission completed in 87.829 simulated seconds at approximately 0.94–0.95 RTF, landed/disarmed and cleaned up.
- Full NVIDIA-rendered mission completed in 86.771 simulated seconds at approximately 0.95 RTF, landed/disarmed and cleaned up. Steady camera samples: approximately 140–143 frames per 5-second reporting window (28–29 FPS). Publisher scheduled 2604 captures with zero queue drops; camera reported zero publication drops/stale frames, one sequence gap and two superseded requests.
- Remaining overhead is not eliminated: instrumented rendered backend commands averaged 0.138 ms, and conservative pacing retains a full timestep after command completion. No busy-wait or catch-up scheduling was added.
- Timing logs: /tmp/terrain-realtime-flight.log, /tmp/terrain-realtime-rendered.log and /tmp/terrain-realtime-camera.log.

## Scope/defaults
Keep 240 Hz timestep, 10 Hz status polling, freshness timeout, RC streaming, flight gains and no-catch-up pacing. Worker shutdown precedes SITL teardown. No firmware or dependency changes.

## Status
Implemented and verified; uncommitted.

## Implementation commit
Pending.
