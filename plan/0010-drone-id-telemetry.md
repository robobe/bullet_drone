# Drone ID in configuration and MCAP

## Goal
Add stable configured drone identity to simulation state and both physics MCAP
schemas/payloads, independently of episode identity.

## Steps and interfaces
- Add optional drone.drone_id to YAML/DroneSettings, default drone_1; validate a
  nonblank string of at most 128 characters. Keep existing construction compatible.
- Append drone_id to DroneState with the same default; physics copies config ID
  into reset/step observations and retains it across episode resets.
- Require drone_id in ground-truth and sensors topic schemas, and populate every
  recorded physics sample from its observation. External FrameIdentity is unchanged.
- Update sample config and focused telemetry/config documentation.

## Verification
Test default/custom/invalid YAML and direct settings, persistence across reset and
step, schema/payload agreement and differing IDs. Use unit recording probes when
the optional service is absent; update real MCAP readback tests and run full suite.

## Scope/defaults
No dependency or physics changes, no recording CLI, no commits or pushes. The
additive required field changes newly emitted MCAP payload schemas; old files keep
their embedded old schemas. Existing renderer-profile identity is unchanged.

## Status
Complete; implemented, verified and committed.

Verification outcome:
- Focused config/identity/telemetry/physics checks: 48 passed.
- Full suite: `uv run --locked pytest -q` → 275 passed, 1 skipped.
- Existing YAML/constructors retain drone_1; custom IDs and the length boundary
  load correctly; blank/nonstring/overlong IDs are rejected.
- Live physics test confirms identity survives stepping and reset while episode
  UUID changes; observation identity remains immutable.
- Recording probe exercises the real adapter with observations from two configured
  drones and verifies schema keys, required drone_id, sensor payloads, frame
  identity and extras. No external writer dependency is needed for these checks.
- Real MCAP readback assertions updated, but integration remains skipped because
  mcap_bug is not installed. Documentation states schema compatibility and unchanged
  renderer-profile identity. Dependencies unchanged; diff whitespace checks pass.

Implementation commit hash: `65830c3a85a7928ddcabf28cb6615be2e28fa187`.
