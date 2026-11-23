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
