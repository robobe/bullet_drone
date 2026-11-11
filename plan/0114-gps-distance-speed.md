# GPS destination distance and faster cruise

## Goal
Log latitude/longitude in degrees, MSL altitude, and distance to destination;
increase the GPS point-to-point cruise target from 3 to 5 m/s.

## Steps and interfaces
Use the existing MAVLink geographic feedback. Report horizontal distance to B in
metres and actual speed in m/s. Replace verbose local ENU/vector output in status
logs with these fields. Increase lookahead from 5 to 8 m so the position gain
allows a 5 m/s command. Preserve braking, acceleration, takeoff/landing limits,
feedback freshness and independent acceptance thresholds.
Live verification at 5 m/s found a 1 m startup cross-track transient. Add a
1 m/s² vector slew limit to the cruise velocity target, reset at transitions;
preserve existing inner controller limits.

## Verification
Update guidance checks for the new speed and braking. Run focused tests and a
complete paced headless SITL flight, checking tracking, clearance and touchdown.

## Scope and defaults
Distance is horizontal GPS/local-tangent distance to B, not 3D slant range.
No dependency, firmware, core interface or renderer changes.

## Status
Implemented and verified: 15 focused tests passed, 1 opt-in live test skipped.
The bounded startup trace reduced cross-track peak from 0.975 m to 0.025 m.
Complete paced headless SITL flight passed: 160.262 s simulation time versus
208.592 s previously, approximately 23% shorter. Actual cruise approximately
4.67 m/s, maximum full-flight cross-track error 0.152 m, landing error 0.019 m,
minimum cruise clearance 9.952 m, terrain contact true, armed false. Shutdown
completed. Whitespace check passed. Renderer was not rerun for this change.

Implementation commit: Pending
