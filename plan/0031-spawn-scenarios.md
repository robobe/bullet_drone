# Select and save drone starting positions in Godot

Status: Complete; committed. Implementation commit: `1163fc93b80b7753cfc7528eeb4ff77718f75dab`.

## Goal and defaults
Click any ground point in the complete 500 x 500 m landscape for Bullet X/Y,
enter world Z (0.015–500 m, millimetre precision), and Apply to create a fresh
stopped episode. Require centre clearance >= terrain triangle height + 0.015 m.
Named presets live in versioned user://spawn_scenarios.json with atomic saves;
Default is read-only (0,0,0.015). Load edits draft only. Identity attitude/zero
velocity and the existing relative 20 m takeoff/warmup (including initial freefall
for elevated starts) remain unchanged. No terrain physical collision or hover
initialization. No commit or push requested.

## Implementation and interfaces
- Dedicated landscape picking collision layer; ray queries in physics process;
  exclude UI clicks. Scenario panel, draft/applied markers and status; actual
  drone follows authoritative physics pose only. Disable edits while running.
- Shared strict SpawnRequest and SpawnState JSON schemas on existing lifecycle
  5557 and telemetry 5555. Bind request to controller, physics, episode and stable
  request ID; require live controller and stopped physics. Deduplicate retries.
- External RC only: initialize replacement caller-owned simulation from immutable
  config before replacing old simulation. On failure preserve old config/sim.
  Successful Apply clears lease/history/sensors/controllers and creates episode.
  Start/Reset reuse applied position. Do not change core reset API.
- Bounded raw native worker slots, main-thread parse/validation. Retry every
  100 ms, 2 s unconfirmed status with reconciliation; block Start/Reset/Apply
  until reconciled. Publish retained authoritative spawn status at 10 Hz even
  stopped, including current episode and latest request outcome; record by default.

## Verification
Focused protocol/schema, malformed/stale/foreign/running/duplicate real TCP,
replacement failure and lifecycle tests. Godot headless picking/frame/clearance,
preset and UI checks; native build; real-display selection/Apply/Start/Reset,
existing flight/collision regressions and MCAP readback. Record outcomes here.

## Verification outcome
- Native debug configure/build passed with pinned dependencies unchanged.
- Full workspace suites passed: 935 tests (messages 124, LOS 81, recorder 68,
  app 36, physics 606, renderer 20). After the final cleanup hardening, all
  9 focused spawn tests passed, including an additional cleanup-failure case.
- Godot headless frame, truck-collision and spawn checks passed. Spawn checks
  include actual UI-excluded screen input far outside the clearing, triangle
  clearance/bounds, versioned presets and failure preservation, running locks,
  acknowledgement/timeout reconciliation and physics restart. The default
  64px headless viewport is scaled explicitly for UI event tests.
- Real-display scene normals/RGB/FOV/UI exclusion checks passed. Real scene-click
  integration selected (10,5,5) m, confirmed a fresh frozen episode, recovered
  after renderer restart, reused spawn on Reset and reached hover near 25.15 m.
  MCAP readback retained 214 spawn statuses; stopped preview opened no bag.
  Report/screenshot: /tmp/bullet-spawn/report.json and spawn-applied.png.
- Existing real-display flight regression passed (799 exact image/source
  associations, 33 degree conversion samples, 49 ms app-loss watchdog).
  Collision regression passed across two episodes, including deliberate first
  report loss/retry, reset and final inactive LOS markers. Reports remain in
  /tmp/bullet-spawn-flight/ and /tmp/bullet-spawn-collision/.
- Git diff whitespace checks passed. No dependencies/system tools installed,
  no core reset API changes, no app/LOS/recorder implementation changes, and
  no commits or pushes. Tests use isolated ports and temporary bags, leaving
  existing workspace services untouched. Presets remain local user data.

Follow-up: [0032](0032-scenario-takeoff-height.md) replaces manual-only ground
selection Z and fixed 20 m climb with automatic terrain Z and scenario takeoff height.

## Commit record

User requested committing all projects after verification. Implementation commit: `1163fc93b80b7753cfc7528eeb4ff77718f75dab`. No push requested or performed.
