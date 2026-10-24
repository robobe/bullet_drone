# 0045: Flight analysis recording

Status: Implemented, verification complete.
Implementation commit: `8cde477426dc12f2c6be12542487a9edd50f8890`.

Goal: Capture every standard flight in MCAP plus per-process diagnostic logs,
without changing tracking behavior or clipping rejection.

Implementation:
- Enable the existing episode-controlled recorder in the standard launch task.
  Start recorder before flight services, retain process logs per launch session.
- Preserve original image/source identity alongside pixels in one queued item.
- Record bounding-box edge margins, touched edges and the visual input identity
  actually processed by navigation, retaining existing controller diagnostics.
- Publish actual ground truth at 100 Hz, available sensor outputs at physics
  sampling rate, resolved physics/app configuration and revisions, wind at 10 Hz.
- Record recorder health and stop outcomes, keep failed files partial, and drain
  final mission diagnostics for 250 ms after stop/fault.
- Add an offline clipping report with nearby frames, errors, commands, wind,
  delays and optional CSV exports. No duplicate live CSV logging.

Interfaces: shared JSON schemas, existing telemetry/app endpoints, paired image
metadata channel and final recorder health. Physics library remains IO-free.
Bounded queues and nonblocking publisher preserve flight-control timing.

Verification: schema boundaries, exact frame association and dropped frames,
all image edges, controller noninterference, run lifecycle and failure paths,
recording overhead at 640x480 camera rate, report readback, regressions.

Defaults/limits: external-RC/LOS workflow, one MCAP per flight, separate process
logs per launch, existing SI units/source stamps. Preserve available filtered
sensor outputs; do not invent unavailable raw readings. Defer detailed force
breakdown and tuning/control changes. Large output remains ignored.

Verification outcomes:
- 183 focused physics, sensors, runner and publication checks passed. The real
  TCP analysis test confirms 241 sensor samples and 101 actual-truth samples over
  one simulation second, independent of selected barometer rendering pose.
- 99 LOS checks (including all four clipping edges), 83 app checks, 146 shared
  message checks and 26 native/pipeline/launcher checks passed.
- All 73 recorder checks passed, plus the added CLI readiness/signal check. After tightening report matching for frame
  sequence reuse across renderer restarts, 33 affected recorder/report/config
  checks passed again. Tests cover paired acceptance, loss/partial files,
  preflight config snapshots, late fault drain and legacy indexed recordings.
- Native extension build, headless frame/FOV and NVIDIA real-scene checks passed.
  Two real-display episodes passed recording integration with final app/LOS
  status, config, wind, truth, sensors, renderer health and exactly one image
  metadata record for each saved image. Artifacts: /tmp/bullet-recording-integration-final.
- A 640x480 RGB8, 30 Hz writer check saved all 60 frames without errors. Queue
  peak 921654 bytes, maximum caller enqueue about 0.11 ms on this machine.
  This measures recorder acceptance/writing, not a long-flight performance claim.
- Legacy report exercised on the latest local October 22 recording; it correctly
  reports no recorded clipping and missing newer analysis topics.

Implemented details: paired image metadata also matches capture time to handle
frame counter reuse and late foreign-episode packets. Configuration caches preserve original source identity,
including snapshots from a preceding stopped episode. A bounded analysis queue
keeps serialization and networking off the physics caller. Finalization errors
retain .partial and process logs take precedence over embedded completeness.

Limits: the user's latest failed flight was not available as a recording. No
long-duration wind/mission tuning or aerodynamic calibration was performed.
Detailed force breakdown and raw readings not exposed by sensors remain deferred.
Controller behavior remains unchanged.

Sibling implementation commits:
- bullet_msgs: `c517f00288452b66f8482e78b58c713dd03b7ee8`.
- bullet_render: `4725e8b0a46637e2fc256b2107e150bd73fa7c92`.
- bullet_app: `08cbf83561c127a459039373ec1d2bee05809f06`.
- los_tracking: `e677833404f3de558c2fe4b92fe8757c61f60da7`.
- mcap_bagger: `507c96e40a93d7dced5fc89534ff6d7b3100c517`.
- Shared workspace: `963445ff2a001ffdecdf9ea6fc97cb045c67ca25`.
