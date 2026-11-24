# Horizontal visual centering

## Goal

Improve roll-only camera centering during forward approach and remove the
physics-timestep mismatch from the image-error derivative.

## Implementation steps

1. Raise the proportional roll gain from 12 to 24 degrees per unit normalized
   horizontal error, retaining the existing derivative gain and roll limit.
2. Preserve camera source frame steps through the vision sample API and use the
   elapsed frame-step time when updating the filtered horizontal derivative.
3. Keep the current forward-pitch schedule, vertical guidance, and target-loss
   behavior unchanged; do not add yaw steering or a centering phase.
4. Update the README, add focused tests for gain, repeated frames, frame timing,
   and roll saturation, then run the rendered example.

## Interface changes

Add a vision sample carrying source frame step and optional target. Keep the
existing target-only accessor. Add an optional frame-step keyword to the mission
command for the image derivative.

## Verification

Run focused red-truck tests and the documented rendered simulation. Compare the
approach `abs(dx)` trend with the previous run and record whether the target is
centered or lost. Inspect shutdown and confirm owned processes are cleaned up.

## Scope and defaults

Roll-only steering; continue forward motion. Initial proportional roll gain is
24 degrees per normalized `dx`; derivative gain, filter time constant, pitch,
vertical guidance, and limits remain unchanged.

## Status

Implemented. Focused checks: 5 passed. Full red-truck file: 16 passed, 3
unrelated existing takeoff/MAVLink fixture failures. Rendered run exited normally
after the 150 s mission deadline triggered landing and disarm; no truck contact
was reported. `dx` improved to about +0.31 near 40 m and about +0.11 after
reacquisition, but the target was lost during descent and the vehicle reached
the ground before reacquiring it. Implementation commit: Pending.
