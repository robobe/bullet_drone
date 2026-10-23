# Center the target horizontally before ending yaw search

Status: Complete; committed. Implementation commit: `fca7f74d8d8fbf6d74c537420d5e4ea8a44c49e5`.

## Goal and defaults
After the existing takeoff and three-second hover, do not end search merely
because a usable target is visible. Use bbox center X minus image width/2;
center within +/-10 pixels before handing off to LOS. No vertical centering or
roll corrections. Without detection keep the existing positive 15 deg/s scan.
With a usable detection use proportional yaw, gain 1.5 /s on atan(error_x/fx),
negative for an image-right target, bounded to +/-15 deg/s. Within the band command
zero yaw rate and require 0.3 simulation seconds centered with measured yaw rate
<=5 deg/s and existing altitude/vertical-speed/level tolerances. Drift resets
settling and resumes centering; loss resumes the original remaining scan.
Preserve the measured full-circle and 34 s total budgets, including centering.
A target already visible after hover must also center/settle before acquisition.
Save the centered heading, preserve filter/thrust state and accept only images
captured after settling for normal LOS acquisition/approach. Yaw holds that
heading during approach. User requested implementation now; tuning follows.

## Implementation and interfaces
App-only center-rate helper, YAW_CENTER phase and revised YAW_SETTLE behavior;
no controller/library, physics, ports, dependencies or wire version changes.
Expose bbox/image centers, pixel error/tolerance, centered flag and actual yaw
reference through existing mission diagnostics and readable status. Update
tests, display/MCAP checker and docs. No commits or pushes requested.

## Verification
Pending: left/right signs, saturation/deadband, visible-but-off-center gating,
centering/settle drift and lost-target resumption, stale/foreign/clipped inputs,
heading/filter continuity and centered-only acquisition, reset/watchdogs; real
sensor-only PyBullet, display target-behind-spawn and visible target cases, MCAP
readback and no-target regression. Keep previous unrelated doc edits intact.

## Verification outcome
- App suite passed: 65 tests, including left/right yaw sign, saturation/deadband,
  visible-but-off-center gating, overshoot correction, centered dwell reset,
  loss resuming the original scan, centered-only handoff, source freshness,
  timeout/reset/watchdog and two sensor-only real PyBullet closed loops.
- LOS library 90 and renderer 21 tests passed (176 tests total). No library,
  core physics, native code, dependency, endpoint or wire schema changes.
- Real display behind-spawn target: centering began with X error -227.5 px;
  LOS handoff occurred at -9 px after settling, heading 179.117 deg. MCAP
  retained 1,306 search/center/settle samples, 812 exact source associations
  and 35 degree conversion checks. App-loss watchdog froze physics in 59 ms;
  restart, Reset/readiness and explicit Stop passed. Report and screenshot:
  /tmp/bullet-centered-found/report.json and yaw-search.png.
- Visible-target display: centered at +3.5 px, completed the 0.3 s dwell, then
  acquired/approached without a blind scan; 837 exact source associations and
  49 ms app-loss watchdog. Report: /tmp/bullet-centered-visible/report.json.
- Hidden-target full-circle display regression passed with search_exhausted,
  frozen physics and inactive LOS telemetry. Report/screenshot:
  /tmp/bullet-centered-empty/report.json and yaw-search.png.
- Reviewed wrapped centering status/pixel-error overlay at
  /tmp/bullet-centered-overlay.png. Whitespace checks passed. Existing
  uncommitted image-flow documentation in los_tracking/docs remains untouched.
- No commits or pushes requested/performed. Restart bullet_app to load this
  revision; named centering gain/tolerance constants remain easy to tune.

## Commit record

User requested committing all projects and pushing to GitHub.

- bullet_app: `e2017bbbaada2c668df3fe1705433c0300168976`
- bullet_drone: `fca7f74d8d8fbf6d74c537420d5e4ea8a44c49e5`
- bullet_render: `c94b9818dae7413f11f2e4830fd576be8f44d1c4`
- los_tracking: `1717c192705372b0ab9cf1169fa3445cbf80db0b`

The los_tracking commit documents image flow and message fields; its runtime code is unchanged.
