# Standalone GPS point-to-point MAVLink example

## Goal
Take off 10 m above point A, fly a 400 m line over the shared external-renderer
terrain, and land at GPS point B. Firmware MAVLink is the navigation feedback;
MSP carries Acro RC commands to Betaflight.

## Implementation steps
1. Add an independent example with local CLI, MSP client, RC encoding, SITL
   lifecycle, MAVLink receiver, geographic conversion, mission and execution loop.
   Import no other examples. Reuse core controllers and renderer integration.
2. Hardcode geographic origin and GPS endpoints in the entry point. Query actual
   collision-mesh elevations and check a 5 m-wide corridor at 0.5 m spacing for
   at least 3 m clearance before launching SITL.
3. Configure GPS-compatible firmware with 20 Hz position/attitude streams and
   1 Hz heartbeat. Reject CLI configuration errors. Decode GPS fix, ENU position
   and velocity and pinned-firmware attitude signs with local freshness checks.
4. Wait for feedback, arm, take off, follow the A–B line with 5 m lookahead,
   settle at B, descend using mapped elevation, confirm stable touchdown, disarm.
   Close telemetry, disarm MSP, stop firmware and release physics/viewer on errors.
5. Add focused tests, independent flight metrics, README and design-menu links.

## Interfaces
New example-owned `GeographicFrame`, immutable `Feedback`, `Telemetry`, `Mission`
and `run_demo` interfaces. No core library or existing example interface changes.
The mission consumes only firmware feedback, mapped elevations, arm status and
caller-owned time. Ground truth is used only for FDM, rendering and verification.

## Scope and defaults
- Origin `(32°, 35°, 100 m MSL)`.
- Start `(31.998941979585357°, 35.001727163237504°)` = local
  `(163.205, -117.321)` m.
- Destination `(32.00074562062305°, 34.998061181190316°)` = local
  `(-183.205, 82.679)` m, exactly 400 m from the start.
- **Scope correction:** choose endpoints against the collision mesh, rather than
  raw heightmap rows. The original west-to-east route cannot clear the terrain
  after a 10 m takeoff. Its reversal cleared the route but the west landing site
  caused a measured 1.818 m slide after disarming. The final diagonal route
  clears its corridor by 8.41 m and has a flatter destination. Preflight now
  rejects destination patches with more than 0.05 m height variation across
  center and four points 0.3 m away.
- Fixed cruise Z about 34.29 m (134.29 m MSL), up to 3 m/s, landing up to 0.3 m/s.
- GPS and known terrain map for landing, no direct rangefinder feedback.
- Core package and renderer allowed, no other-example imports or new dependencies.
- Ideal simulated GPS only, no obstacle avoidance or real-aircraft failsafe.
- Use `uv run --frozen` with existing workspace lock. `--locked` currently reports
  an unrelated workspace lock mismatch; do not rewrite dependency state here.

## Verification
- Unit checks: GPS conversion/model bounds, line correction/braking, captured
  pinned-firmware attitude signs, message units, invalid/stale feedback, landing
  and disarm sequencing, standalone imports and no controller truth access.
- Full suite: 644 passed, 8 skipped. Focused example tests: 14 passed, 1 skipped.
- Headless real SITL: passed, 208.592 s simulation time. Independent landing
  error 0.011 m, maximum cross-track error 0.423 m, minimum measured cruise
  clearance 9.959 m, terrain contact true, firmware armed false. Cleanup passed.
- Rendered real SITL: passed using NVIDIA offload, 208.500 s simulation time.
  Landing error 0.011 m, maximum cross-track error 0.433 m, minimum cruise
  clearance 9.960 m, contact true, armed false. Inspected captured cruise and
  landing camera frames. Camera reported no invalid frames or publication drops.
  Owned SITL/viewer processes exited; port release checked.
- Initial live runs exposed and corrected attitude signs, the actual CLI setting
  `mavlink_pos_rate`, and packet-burst pacing that perturbs the SITL clock.

## Status
Implemented and verified. No commit requested; implementation commit remains Pending.

Implementation commit: Pending

Follow-up: [0114](0114-gps-distance-speed.md) updates cruise to 5 m/s, lookahead to
8 m, and status logs to show geographic position and horizontal distance to B.
The verification figures above record the original 3 m/s implementation.
