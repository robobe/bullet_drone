# LOS 35-degree azimuth envelope and clear mission errors

Status: Complete; committed. Implementation commit: `f6e5e302242388f337fee28aff70690f316d43c6`.

## Goal and implementation
- Raise the default LOS and Bullet profile max azimuth from 30 to 35 degrees
  (symmetric +/-35); keep all other control/timing/safety limits unchanged.
- Retain rejected angular measurements and configured limits in public navigator
  diagnostics, separate from valid filtered bearings. Never control on rejected
  observations or report them as valid LOS estimates.
- Format app console/debug status in readable text with degrees and limits;
  show rejection cause while acquiring and preserve it in timeout messages.
  Emit logs on phase/reason/rejection changes rather than only phase changes,
  without logging every noisy angle sample. Preserve raw machine reason codes.
- READY after stopped feedback must report the physics reason rather than the
  generic 'ready'. Include physics state/reason and acquisition timeout context
  in mission diagnostics/MCAP. Keep existing FAULT latch until a new episode.
- No flight-state, yaw behavior, telemetry port or focused LOS schema changes.
  No commits or pushes requested.

## Verification
Boundary tests within +/-35 and rejection above; rejected diagnostics clearing
and no valid bearing contamination. App formatting in degrees, timeout/rejection
and stop/watchdog messages, state/reason logging keys and mission fault/reset.
Affected library/app/shared/recorder tests and live profile acquisition smoke.

## Verification outcome
- Affected suites passed: LOS 89, app 41, shared contracts 127, recorder 68
  (325 tests). After final formatting/overlay changes, 7 focused status/display
  tests passed. Tests include inclusive +/-35 boundaries, rejection at +/-35.01,
  acquisition/recovery timeout cause retention, stale accepted samples not hiding
  fresh rejections, diagnostic clearing, stop reasons, and no angle-noise log spam.
- Sensor/bbox replay from the user's exact recorded scenario now reaches TRACK
  after 0.141 s with accepted azimuth 31.941 degrees against the 35 degree limit.
  Report: /tmp/bullet-los-35-replay.json. Replay uses recorded measurements and
  validates acquisition; it does not claim a new closed-loop collision outcome.
- Real-display flight regression passed: all takeoff/hover/acquire/approach phases,
  828 exact source associations, 34 degree conversions, 59 ms app-loss watchdog,
  restart/reset and MCAP readback. Report: /tmp/bullet-los-35-flight/report.json.
- Reviewed wrapped debug error overlay: /tmp/bullet-los-rejection-overlay.png.
  Whitespace checks passed. No native source, dependency, core physics, port or
  focused LOS wire-schema changes. No commits/pushes; restart bullet_app to load.

## Commit record

User requested committing all projects.

- los_tracking: `10b73e11fc49b983b0eed3c4568307703f91ef88`
- bullet_app: `eaa45c948432e72a08e3a7af9904f74e6ffa661f`
- bullet_drone: `f6e5e302242388f337fee28aff70690f316d43c6`

LOS (89) and app (41) tests passed again before committing. No push requested or performed.
