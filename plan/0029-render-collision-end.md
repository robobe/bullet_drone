# Renderer collision and scenario completion

Status: implemented, verified and committed. Implementation commit: `aecc8312cea2cea622fd08b893b51d9be794ffbb`.

## Goal and decisions
Godot detects drone-envelope contact with any red-truck part; physics confirms
success/target_hit and freezes the episode. The app latches COMPLETED, the renderer
shows the result and the recorder saves the result before finalization. Keep all
processes open; Start/Reset clears termination. No terrain/scenery collisions,
physical response, landing or automatic reset. Commits were subsequently
authorized by the user; pushing remains outside scope.

## Implementation
- Truck-part convex detection colliders; 0.42 m drone sphere; initial overlap plus
  swept queries on the main physics thread, independent of GPU readback. Clear
  sweep/latch on episode/stream changes; never sweep across resets.
- Shared bounded CollisionReport and ScenarioEnd contracts; collision reports on
  5557, confirmed scenario/ended on 5555. Retry every100ms, 1s acknowledgement
  deadline then ordinary Stop/error. Native workers buffer raw bounded data only.
- Physics accepts current-controller/current-episode fresh reports matching a
  bounded 1s source history before the next step/watchdog decision; first end wins.
  Revoke RC lease and preserve contact source separately from final frozen source.
- Simulation state v2 carries terminal_result; retain v1. Repeat confirmed result
  until new Start/Reset for late subscribers. App latches COMPLETED, stops LOS and
  sends inactive/disarmed intent; stopped feedback cannot erase the result.
- Recorder deduplicates event IDs, enqueues confirmed result through its bounded
  writer and drains final messages for250ms before closing. Add recorder topics.
- Existing square examples keep legacy lifecycle behavior; no new endpoints.

## Verification
Completed on Linux/Godot 4.7, with pinned dependencies unchanged:
- Full workspace regression: 882 passed (bullet_msgs88, los_tracking81,
  mcap_bagger68, bullet_app28, bullet_drone598, bullet_render19).
- CMake debug configure/build and all Python source/wheel builds passed.
- Godot frame/FOV and real-display scene/pixel checks passed.
- Godot collision geometry checks passed: overlap, near miss, envelope contact,
  tilted truck, fast swept crossing, duplicate latch and reset boundary.
- Shared schema/validator and real TCP tests cover malformed/foreign/future/stale
  reports, source-history matching, live controller, duplicate/first-end behavior,
  completion latching, recorder state/event race, deduplication and final drain.
- Real-display collision probe drove deterministic observations through production
  external-RC wiring. Two episodes recovered from deliberately dropped first
  reports: 4 proposals and exactly2 confirmed events in2 finalized bags. Final app
  COMPLETED diagnostics were recorded. Physics stayed frozen; late app restart
  recovered COMPLETED. A rejected Start without app readiness preserved success;
  Reset/new Start produced a separate event ID and recording.
- A separate all-reports-rejected probe verified the1s acknowledgement deadline,
  ordinary Stop and no false success/end event.
- Existing real PyBullet takeoff/hover/LOS integration still passed, with828 exact
  bbox/pose associations, reset/restart checks and app-loss watchdog freeze49ms.

Evidence: `/tmp/bullet-collision-tests.log`, `/tmp/bullet-build.log`,
`/tmp/bullet-collision-python-build.log`, `/tmp/bullet-collision/report.json`,
`/tmp/bullet-collision/completed.png`, `/tmp/bullet-collision-timeout/report.json`,
`/tmp/bullet-flight-with-collision/report.json`. Temporary bags were read back then
removed. Implementation committed on user request; no pushes performed.

The deterministic collision probe substitutes observations for physical motion;
production geometry, native transport, physics-loop acceptance/freeze, app and
recorder are exercised. Automatic LOS navigation all the way into the truck is
not an acceptance test. There is no physical impact response or rewind.

## Implementation commits

- bullet_msgs: `f3c1b3c9ec14249a91c3921c8d813cecabe1e306`
- bullet_app: `4b1899f761c593e0d06d5d8f629f5ba240b6a108`
- bullet_drone: `aecc8312cea2cea622fd08b893b51d9be794ffbb`
- mcap_bagger: `3248fa10e1dca2131ae7aec4baf94a9f2cb02636`
- bullet_render: `473933e5cc9f87b85ad54bc42738dc53a122296f`
- los_tracking: unchanged; no new commit needed.
