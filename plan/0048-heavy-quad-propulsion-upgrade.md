# Heavy quad propulsion upgrade

## Goal

Increase heavy_quad maneuver authority from a 1.47:1 to a 2.5:1 thrust-to-weight baseline using an 18-inch propeller and a larger frame envelope.

## Implementation

- Update the packaged heavy_quad URDF geometry, rotor spacing, propeller radius, thrust capacity and inertia.
- Keep the existing provisional motor/propeller coefficients until measured hardware data is available.
- Update the matching mission profiles and Betaflight PWM calibration.
- Extend vehicle, configuration and mission tests for the new capacity and geometry.

## Verification

- Validate the packaged URDF and all heavy profiles.
- Run focused vehicle, physics, mission and external-runner tests.
- Run a heavy_quad internal smoke check and Betaflight SITL smoke check where available.

## Scope and defaults

- 5 kg vehicle, 18-inch props, 0.34 m arm radius and 2.5:1 total thrust-to-weight.
- Battery, ESC current, motor KV and thermal behavior remain outside the current simulator model.

## Status

Implemented and verified with focused vehicle, physics and runner tests.

## Implementation commit

`4de86bd`
