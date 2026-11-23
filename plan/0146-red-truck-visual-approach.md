# Red truck visual approach

## Goal

Add a runnable Betaflight Acro example that uses the rendered camera SHM frames
to detect the red truck, centers it by image error, descends during a fast glide,
and succeeds only on renderer-confirmed truck contact.

## Implementation

- Add an example-owned mission, SHM/contact renderer adapter, CLI, SITL loop,
  diagnostics and README. Take off to 30 m AGL, yaw-search for 30 s, then use
  normalized camera-centroid dx/dy for roll/pitch/thrust corrections while
  ramping forward speed to 3 m/s and descending. On vision loss brake and hold
  altitude; on search timeout land and disarm.
- Add an additive, stream-stamped visual-contact message on the renderer control
  endpoint. Godot uses the existing swept red-truck detector; Python accepts only
  contacts from its active render stream and ends successfully on contact.
- Add the example to the root README and VS Code example picker.

## Interfaces

- Add a strict `render/drone_1/contact` protocol contract containing event ID,
  stream ID, source pose step, target ID and drone position.
- Add native/GDScript publication for that contact event. Preserve existing
  telemetry, camera and request/reply interfaces.

## Verification

- Focused tests cover detector centroid/thresholds, normalized image errors,
  command directions, target-loss reacquisition, search timeout/landing, and
  fresh active-stream contact over a real TCP socket.
- Python checks (19 passed), CMake/native build, Godot swept-contact test, and
  packaged renderer export passed. The rendered SITL flight remains unverified
  because this session has no display.
- The broader `test_zmq_render.py` run had one pre-existing fixture failure: its
  dummy observation lacks the sensor fields required by `from_observation`.

## Defaults and status

- Use the existing camera profile and Betaflight SITL setup; NumPy in the render extra accelerates centroid detection.
- Target contact is the only success condition. Image size is not used for depth.
- Status: Implemented; live visual flight verification pending. Implementation
  commit: Pending.

## Camera throughput follow-up

- Capture and detection run in the vision worker and publish to a capacity-one
  result queue. The main loop consumes without waiting and expires stale results.
- Bound descriptor draining and copy only the newest SHM frame. Replace Python
  pixel iteration with NumPy operations to reduce GIL contention.
- Verify queue replacement, freshness, cleanup, detector results and detector timing.
- Follow-up verification: 9 focused tests passed. Ten-frame synthetic 640x480
  black-image benchmark: approximately 32 ms/frame before, 13 ms/frame after.
  Full renderer/SITL FPS and MSP timeout resolution remain unverified.

## SITL timing follow-up

- Poll MSP status in an owned worker so status round trips do not pause FDM.
- Space physics packets by at least one timestep instead of catching up after a
  delay. SITL derives simRate from packet arrival spacing; bursts distort its clock.
- Preserve stale-status rejection and fail on worker errors; retain RX failsafe.
- Focused checks: 10 passed, including a blocked status reply while the caller
  continues reading the previous sample. Implementation commit: Pending.
- Real SITL/PyBullet with a no-camera renderer stub ran for 40 simulation seconds,
  climbed to about 30 m and began yaw search without unexpected disarm or MSP
  timeout. It ended at the configured duration limit. Full rendered flight remains
  unverified; this run supports the timing fix but does not prove all RX-loss causes.

## Approach response and diagnostics

- Increase approach speed to 6 m/s, descent to 3 m/s, tilt limit to 30 degrees,
  forward pitch to 25 degrees, pitch slew to 45 degrees/s, vertical image trim
  to 8 m and pitch image gain to 20 degrees. Preserve landing speed at 1.5 m/s.
- Raise altitude PID velocity output limit to 3 m/s (previously capped at 1)
  and acceleration limit to 5 m/s². Use configured ACTUAL rate inversion.
- Log all phase transitions with colors and normalized target error plus control
  diagnostics at 10 Hz wall time. Queue the CLI log sink to avoid terminal I/O
  in the physics loop. Verify descent authority and diagnostic cadence.
- Focused suite: 12 passed. Rendered interception tuning remains unverified.
  Implementation commit: Pending.

## Approved speed and FPS plan

- Take off at up to 4 m/s, decelerating near 30 m. Track at 10 m/s with
  overspeed braking; preserve approach descent and landing limits.
- Pace at actual FDM send boundary, including work within the timestep and
  scheduling from actual sends after missed deadlines; never catch up in bursts.
- Report wall-time RTF, FDM Hz, scheduled capture FPS, unique SHM frame FPS,
  and detector FPS every second. Preserve worker queue and 10 Hz target logs.
- Verify controller limits, pacing with a fake clock, and a rendered SITL run
  where a display is available. Implementation commit: Pending.
- Implemented. Fifteen focused tests passed, including takeoff/deceleration,
  descent preservation, forward overspeed braking, and fake-clock packet spacing.
- Real renderer + SITL ran for 35 simulation seconds, reached a measured 4 m/s
  climb and entered tracking, without MSP timeout or unexpected disarm. Typical
  observed RTF was 0.95–0.97x with camera/detector rates around 28–29 FPS. The run
  lost the target and entered reacquisition; it ended at the configured duration
  limit. Truck contact is not validated. Implementation commit: Pending.

## Camera-bearing interception tuning

- Failure trace: target dy grew to +0.858 before disappearing while the vehicle
  descended at its 3 m/s cap and retained forward motion. Timed descent did not
  match the camera bearing.
- Replace timed descent/image pitch addition with a calibrated camera ray rotated
  into world coordinates using measured attitude. Request a 10 m/s velocity along
  that ray; control horizontal velocity with bounded roll/pitch and map requested
  vertical velocity through the existing altitude cascade. Limit approach vertical
  speed to 8 m/s; preserve 4 m/s takeoff and 1.5 m/s landing.
- Use renderer FOV and aspect ratio. No truck world coordinates or depth lookup.
- First real rendered SITL run: confirmed contact at 21.892 simulation seconds,
  near (46.91, 0.001, 5.70) m, without target loss during approach.
- Regression coverage includes camera attitude compensation and ray-derived
  descent. Implementation commit: Pending.

- Repeat rendered run confirmed contact at 21.967 s near (46.96, -0.003, 5.67) m.
  Both approaches retained the target. All 16 focused tests passed.

## Sensor-only control completion

- Package-owned MAVLink receiver thread and copied SITL/profile, GPS disabled.
  ATTITUDE and VFR_HUD altitude at 20 Hz, independent freshness checking,
  finite-value validation, filtered barometer differentiation for vertical speed.
- Mission accepts only time, arm status, image error and explicit sensor sample.
  No simulator position, orientation or horizontal velocity enters control.
- Camera dx commands roll, dy corrects forward pitch. Forward pitch reduces when
  lateral error is large and is bounded between 5 and 30 degrees. This maximizes
  allowed pitch within a visibility heuristic, not measured forward speed.
- Removed obsolete horizontal-velocity/ray tests. Vertical PI retuned to 2/1
  for delayed barometer feedback. Full sensor flight verification in progress.
- User requested more tolerant LOS entry: altitude within 1 m, vertical speed
  within 1.5 m/s, held for 0.3 s. This changed rendered takeoff-to-search from
  32.2 s to 12.32 s, immediately acquiring the truck.
- Fixed unconditional descent after target moved above center: vertical demand
  is now 2 + 6*dy m/s down, allowing recovery upward with negative dy.
- Further camera tuning: roll uses filtered image-error derivative damping,
  forward baseline 18 degrees, vertical image pitch gain 10 degrees, and descent
  request 3 + 4*dy m/s. Only camera error and its change enter this guidance.
- Final image roll gains: P=12 degrees per normalized dx, D=30 degree-seconds,
  derivative filter 0.2 s. Sixteen focused tests pass, including stale-altitude
  rejection despite fresh attitude and tolerant takeoff-to-LOS transition.
- Two real rendered MAVLink-only sensor-control runs achieved truck contact at
  19.700 s and 19.738 s. LOS entry occurred at 12.433 s and 12.471 s. No target
  loss, unexpected disarm, or telemetry timeout in these two final runs.
- Package-owned telemetry, SITL and profile are wired. GPS disabled. Controller
  contains no simulator truth references. README diagrams rendered successfully.
  Status: Implemented and validated at the current launch. Commit: Pending.

## 40 m takeoff tuning

- Raise takeoff altitude from 30 m to 40 m while preserving the 4 m/s climb cap
  and existing settling criteria.
- The first rendered 40 m run entered approach but lost the target at 22.225 s,
  at 9.1 m altitude with the target at the bottom of the image. Increase only
  the approach baseline descent request from 3 to 3.5 m/s; preserve the image
  correction, limits, attitude tuning and landing behavior.
- Update focused takeoff tests and user-facing parameter documentation, then
  verify with focused tests and a rendered SITL run. Commit: Pending.
- Verification complete: all 16 focused tests passed. The rendered MAVLink-only
  run retained the target and reached renderer-confirmed contact at 22.225 s.
  Status: Implemented and validated. Commit: Pending.

## Altitude-aware image guidance

- Preserve the current 20 m takeoff default and the validated 40 m behavior.
- Scale the camera `dy` pitch correction from half gain at 20 m to full gain at
  40 m. Scale base descent from 3.0 to 3.5 m/s over the same range, while keeping
  full `dy` vertical authority so a target above center can reverse the descent.
  Preserve controller limits and landing behavior.
- Add a focused comparison at 20 m and 40 m, update parameter documentation,
  and verify both focused tests and rendered 20 m behavior. Commit: Pending.
- The first attempt also halved vertical `dy` authority. It failed because a
  target above center could not cancel base descent, losing the target at 0.7 m.
  Keeping full vertical `dy` authority and scaling only pitch fixed the cause.
- Verification complete: all 17 focused tests passed. The rendered 20 m run
  reached renderer-confirmed contact at 17.167 s and 4.48 m altitude. The 40 m
  gain endpoints remain unchanged and covered by the focused comparison.
  Status: Implemented and validated. Commit: Pending.
