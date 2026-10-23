# App flight channel and takeoff/LOS mission

Status: implemented, verified and committed. Implementation commit: `fac9ee367cf79f7a4a11fbd88f12c925467022cf`.

## Goal and scope
Godot Start begins an app-owned mission: sensor warmup, arm Angle mode, climb
20 m above the initial barometer datum, hold altitude/heading for three settled
seconds, wait at most five seconds for a fresh post-hover bbox, then LOS acquire
and approach. No XY truth control, collision detector or landing. Commits were subsequently
authorized by the user; pushing remains outside scope.

## Implementation steps and interfaces
1. bullet_msgs: versioned RC/config/feedback/mission contracts, optional command
   endpoint, stamped-pose/image association while preserving legacy formats.
2. bullet_drone: separate caller-owned external-RC example, bounded input,
   episode ownership/readiness, expiry watchdog and applied-command feedback.
3. los_tracking: public altitude/heading hold and typed validated handoff snapshots.
4. bullet_app: isolated image worker, 100 Hz mission thread, readiness/arming
   confirmation, takeoff/hover/target/LOS state machine, telemetry and cleanup.
5. Native renderer: carry source identity/measurement time into descriptors;
   preserve binary SHM header, strict legacy compatibility and thread boundaries.
6. Configure tasks, recorder topics and docs; validate sockets, clocks, reset,
   closed loop, real renderer pixels and process-loss watchdogs.

## Defaults
Physics 240 Hz, feedback 120 Hz, commands 100 Hz. Maximum packet life 100 ms;
LOS expiry may shorten it. Ports 5555 telemetry, 5556 image, 5557 Godot lifecycle,
5558 app results, 5559 app flight commands. Climb 1 m/s; settle +/-0.25 m,
|vz| <=0.2 m/s, roll/pitch/heading error <=5 degrees continuously for 3 s.
Takeoff/settle timeout 60 simulation seconds; target wait 5 simulation seconds.
Wall monotonic clocks govern expiry; simulation time governs mission durations.
Stopped physics freezes; fresh Start/reset is required after fatal faults.

## Verification
Completed on Linux with Godot 4.7, pinned dependencies unchanged:
- Shared workspace check: 865 passed (bullet_msgs 77, los_tracking 81,
  mcap_bagger 65, bullet_app 27, bullet_drone 596, bullet_render 19).
- CMake debug configure/build and all Python package wheel/source builds passed.
- Godot headless frame/FOV and real-display scene/pixel checks passed.
- Sensor-only PyBullet test reached 20 m, continuous settled hover and the
  five-second bbox timeout. Focused tests cover applied arming, hover dwell
  reset, post-hover capture identity, continuous handoff and fresh episodes.
- Real TCP tests cover exact topic matching, malformed/stale/duplicate/foreign
  input, readiness, ownership, latched withdrawal and expiry.
- `bullet_render/tools/check_app_flight.py` reached LOS approach through actual
  rendered detector frames. MCAP readback contained all seven selected topics
  and 828 exact bbox/pose source timestamp associations. Bearing/rate diagnostics
  were valid and nonzero approach pitch was commanded.
- Hard app loss froze physics with rc_expired in 0.059 wall seconds. Start was
  rejected without readiness; a replacement app did not resume the frozen
  episode. Reset and fresh Start succeeded; explicit Stop finalized a second bag.

Logs: `/tmp/bullet-all-tests.log`, `/tmp/bullet-build.log`,
`/tmp/bullet-python-build.log`; integration report/logs:
`/tmp/bullet-app-flight/`. Temporary integration bags are removed.

Legacy pose v2/image v1/bbox v1 and square tasks remain supported. New pose v3,
image descriptor v2 and bbox v2 reuse the source stamp; binary SHM remains v1.
No system tools installed or dependencies upgraded. Implementation committed
on user request; no pushes performed.
RawImage MCAP records still use arrival timestamps; bbox v2 and pose v3 retain
exact source association, not an implicit arrival-order pairing of RawImages.
Collision reporting and landing remain outside this milestone.

## Implementation commits

- bullet_msgs: `f525b62a7e218fc0c229919f06b042861588657a`
- los_tracking: `f9ef53f01f94c49bbbd498e566778d8f72019cf9`
- bullet_app: `87491734a43a69814003dcf8bfca9d29420f23d1`
- bullet_drone: `fac9ee367cf79f7a4a11fbd88f12c925467022cf`
- mcap_bagger: `7e36e8cd149be73c59840ccfd24639c41ae15a9c`
- bullet_render: `66370779b64f16abfb85ccdd104c17b5c905bc71`

Parent workspace files (including bullet.code-workspace and README.md) are
outside these Git repositories and remain unversioned.
