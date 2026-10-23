# Yaw target search before LOS acquisition

Status: Complete; committed. Implementation commit: `0861c8cfd6d0442773998500200092bd2660d87b`.

## Goal, defaults and behavior
After the three-second settled hover, check for a fresh usable bbox (at most
100 ms). If absent, search in positive yaw at 15 deg/s while holding altitude
and level roll/pitch. Count a single net 360-degree sweep from measured,
unwrapped attitude. A fresh, unclipped, aligned target inside the existing LOS
angle limits stops search. Hold the detected heading and settle for 0.3 simulation
seconds with yaw speed <=5 deg/s, heading error <=2 deg and existing altitude,
vertical speed and attitude tolerances. Losing the candidate resumes the original
remaining sweep. Save the new heading and hand off to normal LOS acquisition
using images captured after settling. A visible target skips the scan. Full
circle without detection latches search_exhausted; a 34 simulation second total
search deadline handles stalled rotation/settling. Existing lifecycle, collision,
sensor freshness and RC watchdog behavior remains authoritative.

## Implementation and interfaces
- Public hold heading setter, timestamp-aligned attitude lookup and optional
  bounded yaw-rate command override; preserve legacy defaults and handoff state.
- App measured-yaw tracker and YAW_SEARCH/YAW_SETTLE mission phases. Simulation
  time for motion/durations; source monotonic time for image/attitude alignment.
- Search progress, rates, elapsed time and rejection cause in mission diagnostics
  and readable console/overlay. Focused LOS stays inactive until acquisition.
- No wire versions, ports, dependencies, core physics or native production edits.
  Update documentation and isolated real-display test tooling. No commit/push.

## Verification
Pending: hold API validation, unwrap/reverse/duplicate sampling, target bypass,
stale/foreign/clipped/outside/unaligned rejection, settling reset/lost candidate,
heading/filter continuity, no-target full circle, stalled timeout, reset, lifecycle
and watchdog tests; sensor-only PyBullet, affected suites and real-display search,
no-target and existing mission/MCAP regressions. Record results below.

## Verification outcome
- Affected suites passed: LOS 90, app 55, shared messages 127, recorder 68,
  renderer 21 (361 tests). App checks include two real PyBullet sensor-only
  closed loops: no-target measured full circle and behind-spawn target
  detection/braking/acquisition/approach with new-heading continuity.
- Deterministic tests cover unwrap/reversal/duplicates, visible-target bypass,
  bounded missing-frame check, clipping/envelope/foreign/stale/unaligned rejection,
  continuous settle reset, candidate loss with retained sweep/deadline, search
  timeout, new-episode reset, stale feedback, heading/yaw API validation and
  inactive focused LOS telemetry. Godot headless frame/FOV checks passed.
- Real display: a terrain-cleared spawn at Bullet X=100 m with 6 m selected
  takeoff searched for the initially behind-spawn truck, settled at heading
  146.011 deg, acquired and approached. MCAP retained 1,045 search/settle status
  samples, 736 exact source associations and 35 LOS degree conversions; app
  loss froze physics in 49 ms. Restart/reset/readiness and explicit Stop passed.
  Report/screenshot: /tmp/bullet-yaw-found-final/report.json and yaw-search.png.
- Hidden-target display run completed a measured 360.040 deg circle in
  24.779 simulation seconds, latched search_exhausted, withdrew RC and kept
  physics frozen; MCAP retained 2,477 search samples with no active LOS.
  Report/screenshot: /tmp/bullet-yaw-empty/report.json and yaw-search.png.
- Existing visible-target display regression bypassed search and reached LOS;
  828 exact source associations, 35 degree conversions and 57 ms watchdog.
  Report: /tmp/bullet-yaw-default/report.json. Reviewed the wrapped search/error
  overlay in /tmp/bullet-yaw-overlay.png and the real scene screenshot.
- Hardened the test probe screenshot helper to await GPU readback outside
  SceneTree._process: its synchronous bool return controls test-tree lifetime.
  Real search screenshots now preserve the running test process. Distant
  scenario setup uses the existing panel, terrain clearance and Apply handshake;
  no production spawn or native behavior changed.
- No dependencies, wire versions, ports or native production sources changed.
  No commits/pushes or system-tool installation. Whitespace checks passed.
  Core physics suite/native rebuild not repeated: runtime physics/native code
  are unchanged; real-socket/display regressions exercised the installed build.

## Commit record

User requested committing all projects after implementation and verification.

- los_tracking: `deb8c66365975ee5b235f5906661ebd141115536`
- bullet_app: `967cdf256fab67f3471085db7fbae8c3ce5f9889`
- bullet_drone: `0861c8cfd6d0442773998500200092bd2660d87b`
- bullet_render: `95cdd2650b852cc43d031813f1d30a8ed19a07ce`

Implementation message: `yaw target search version1`. No push requested or performed.
