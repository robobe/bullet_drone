# 0043: Physical rotor vibration and its effect on tracking

Status: **not-imp** (not implemented).
Implementation commit: Pending.

## Summary

Implement rotor vibration in `bullet_drone` physics. Rotor forces will change
the drone's true position and attitude; sensors and tracking will respond through
their existing inputs.

Preserve the current routing: ground-truth position, filtered barometer altitude,
and filtered IMU-derived attitude. Validate the model through an offline tracking
experiment first, because resolved rotor frequencies are too expensive for the
current live loop without further timing work.

Use physically defined but uncalibrated test amplitudes. This will be a
rotor-disturbance model, not a claim to reproduce a particular aircraft's recorded
flight.

## Physical model and configuration

Add a focused `forces/rotor_vibration.py` component with two effects:

- Rotating mass imbalance: per-rotor lateral force with magnitude
  `unbalance_kg_m * angular_speed^2`, rotating at shaft frequency.
- Axial thrust ripple: a sinusoidal fraction of each rotor's existing axial
  thrust, at blade-pass frequency. This adds vertical motion and roll/pitch moments.

The imbalance equation follows established rotor-balancing models:
[Rotor-balancing reference](https://rotorlab.tamu.edu/me459/Rotor%20Balancing/2016%20TPS%20Tut%20ROTOR%20BALANCING%20Pavelek.pdf).

Add an optional `forces.rotor_vibration` block:

| Field | Default |
| --- | --- |
| `enabled` | `false` |
| `unbalance_kg_m` | Four zeros |
| `thrust_ripple_fraction` | Four zeros |
| `blade_count` | `2` |
| `seed` | `3` |

Require finite nonnegative imbalance values, ripple fractions in `[0,1)`, a
positive integer blade count and an integer seed.

Track continuous rotor phases using actual speeds and physical spin directions.
Seed initial phases reproducibly on reset. Stopped rotors produce no vibration
forces.

Add the disturbance to the existing forces applied at rotor locations. Let
PyBullet generate the corresponding moments; do not apply those lever-arm torques
again. Do not add a second copy of this vibration to sensor readings.

## Internal sampling and public interfaces

- Add `simulation.integration_substeps`, a positive integer defaulting to `1`.
- Keep existing public step identities, control updates and publication rates
  unchanged.
- Extend the physics step with an optional substep observer, used by the
  environment to sample IMU and update attitude estimation after each internal
  integration step.
- Use unique internal sample identities. Map the final IMU/attitude output back
  to the completed public observation identity, preserving existing message formats.
- Keep barometer sampling at the public physics rate so its configured
  noise/filter behavior remains comparable.
- Run existing IMU filtering before selecting the public output. Publish the
  interval average of filtered IMU measurements; publish the attitude estimate at
  the interval endpoint.
- Preserve the existing execution path when substeps are `1` and vibration is
  disabled.
- Validate resolution against the vehicle's maximum rotor speed and blade count,
  requiring at least 16 internal samples per highest modeled vibration cycle.
  Use 72 substeps (17,280 Hz internally) for the bundled vehicle.
- Leave the supplied live configuration disabled. Until live timing support is
  implemented, reject high-rate integration in the external-RC launcher with a
  clear instruction to use the offline experiment.

## Offline tracking experiment

Add `tools/check_rotor_vibration.py`, using the current physics YAML and existing
mission/controller code without network transport.

Run three repeatable cases with identical sensor seeds and internal sampling:

| Case | Imbalance per rotor | Axial ripple |
| --- | --- | --- |
| Baseline | `0` | `0%` |
| Mild | `1e-8 kg*m` | `1%` |
| Stronger | `5e-8 kg*m` | `5%` |

Use a deterministic simulation clock with existing controller, feedback and image
schedules. Project a fixed world target into synthetic bounding boxes using the
currently configured render-pose routing.

Run until ten simulated seconds after first tracking entry, mission fault, or a
90-second total limit. Report outcomes without forcing any case to fail.

Write JSON summaries and public-rate CSV traces containing:

- Ground-truth position/attitude variation and sensor-estimation errors.
- LOS bearing error/rate, target rejection and command saturation.
- Time to tracking, time spent tracking, recovery events and fault reasons.
- Actual vibration frequencies and simulation runtime.

This separates measurable physical disturbance from its eventual tracking effect.

## Verification and limits

- Verify force magnitude, rotor phase/spin direction, blade-pass frequency, reset
  repeatability and zero-speed behavior.
- Verify forces act at rotor locations without duplicated torque.
- Check substep timestamps, sensor filtering, public identities and unchanged
  disabled behavior.
- Compare 72- and 144-substep results in a fixed-speed hover case; require
  vibration RMS metrics to agree within 5%.
- Run existing sensor, physics, telemetry, hover, search and LOS regressions.
- Demonstrate and report the baseline/mild/stronger offline tracking outcomes.

A rigid-body model may show measurable acceleration vibration but extremely
small position or attitude oscillations. Do not amplify pose artificially to
make tracking degradation visible.

Flexible-frame resonance, sensor-mount dynamics, camera exposure effects, motor
lag calibration and live real-time integration remain outside this first
implementation.

No implementation or verification of this plan has been performed. Saving this
plan does not authorize implementation, committing or pushing.
