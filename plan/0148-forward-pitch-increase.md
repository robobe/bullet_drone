# Forward pitch increase

## Goal

Increase forward pitch during camera-guided approach to encourage higher forward
speed while preserving camera alignment guidance and existing safety limits.

## Implementation steps

1. Raise the forward-pitch baseline from 18 to 22 degrees.
2. Update README descriptions and diagrams' explanatory text.
3. Run the focused red-truck tests and inspect the diff.

## Interface changes

None.

## Verification

Run focused tests and check documentation values against the controller.

## Scope and defaults

Only the red-truck approach pitch tuning and its documentation change. Maximum
tilt, pitch slew, image corrections, and altitude guidance are unchanged.

## Status

Implemented. Focused tests were attempted; 14 passed and 3 failed in existing
takeoff-threshold and MAVLink-registration fixtures unrelated to pitch. The
forward-pitch baseline and README now agree at 22 degrees. Implementation
commit: Pending.
