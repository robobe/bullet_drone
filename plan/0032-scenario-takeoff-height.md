# Scenario takeoff height and terrain-based World Z

Status: Complete; committed. Implementation commit: `1163fc93b80b7753cfc7528eeb4ff77718f75dab`.

## Goal and implementation
- Add Takeoff height (m) to the renderer scenario dialog: relative climb above
  initial barometer datum, default 20 m, finite range 0.1–100 m, 0.1 m steps.
  Apply atomically selects position and takeoff height while stopped; Start/Reset
  reuse them. Save/load both in presets, migrate v1 presets with height 20 m.
- Picking a new terrain point automatically fills World Z with triangle terrain
  height + 0.015 m clearance, rounded upward to 0.001 m. Manual adjustment after
  picking remains available. Load preserves stored Z and revalidates terrain.
- Shared spawn request/state v2 add required takeoff_height_m; v1 remains accepted
  with default 20 m. Feedback v2 carries the applied per-episode height so the app
  can select its goal from the first feedback without config/start ordering races.
  App retains v1 feedback compatibility with 20 m default. MCAP schemas accept both.
- External runner owns configuration; no core physics or LOS API changes. Native
  workers retain bounded raw slots and Godot main-thread validation. No commits.

## Verification
Shared validation/schema compatibility and invalid heights, protocol deduplication
with changed height, failed initialization preservation, mission goal/reset and
real physics takeoff. Godot preset migration/round-trip and terrain auto-fill,
UI running/pending locks, native build/TCP checks. Real display scenario Apply,
Start/Reset and MCAP readback verify a nondefault height; capture screenshot.

## Verification outcome
- Native debug configure/build passed. Shared messages 127, app 37, recorder 68,
  renderer 21 and focused physics/control/transport 32 tests passed (285 total).
  Includes v1/v2 schemas and invalid heights, per-episode mission goal latching,
  legacy 20 m default, atomic height/position selection and duplicate handling.
- Godot headless spawn checks passed, including actual far-landscape picking,
  automatic terrain/clearance Z, preset v2 round-trip, v1 migration to 20 m,
  invalid-height/write/malformed-file failures and running/pending locks.
- Real-display integration passed: screen pick selected (10,5,0.015) m without
  manual Z, selected takeoff 6 m, fresh stopped Apply, renderer restart recovery,
  Reset reuse, and hover at 6.204 m. MCAP readback found 76 spawn v2 statuses,
  each carrying the selected 6 m climb; feedback v2 carried the same value.
  Report and reviewed screenshot: /tmp/bullet-scenario-takeoff/report.json and
  spawn-applied.png. Bags were temporary and isolated ports left services alone.
- The first integration assertion assumed exact float equality for rendered Z;
  updated the test tolerance to account for Godot Vector3 float32 precision at
  0.015 m. Production source pose/configuration retains its exact supplied value.
- Updated README/contracts/design docs. No dependency changes, core/LOS changes,
  commits or pushes. Restart renderer, physics and app to load this revision.

## Commit record

User requested committing all projects after verification. Implementation commit: `1163fc93b80b7753cfc7528eeb4ff77718f75dab`. No push requested or performed.
