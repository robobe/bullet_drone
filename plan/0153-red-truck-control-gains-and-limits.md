# Red truck control gains and limits

## Goal

Explain the visual-servo control strategy, gains, derivative filtering and
actuator/target limits in the red-truck README.

## Implementation steps

1. Describe the camera-error feedback strategy and each image-guidance gain.
2. Explain the derivative low-pass filter and its 240 Hz update behavior.
3. Document pitch, roll, velocity, body-rate and thrust limits from the code.
4. Check the documented values against controller settings.

## Interface changes

None.

## Verification

Compare the README values with controller constants and implementation. Tests
are not run.

## Scope and defaults

Documentation only; no control values change.

## Status

Implemented; README values checked against controller configuration and limit
logic. Tests were not run. Implementation commit: Pending.
