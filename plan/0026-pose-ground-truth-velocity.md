# Ground-truth velocity in ZMQ pose telemetry

## Goal

Publish Bullet's existing `DroneState.linear_velocity_m_s` in each
`render/drone_1/pose` message on telemetry TCP port 5555, for PlotJuggler and MCAP.

## Steps

1. Add required `linear_velocity_m_s` = world `[vx, vy, vz]` in m/s to the
   shared pose schema. Use pose schema version 2 because strict consumers reject
   extra/missing fields; config, control and image versions remain unchanged.
2. Serialize the existing ground-truth vector in active and held pose packets.
   Update native Godot validation and its fixtures, retaining scene transforms.
3. Verify nonzero signed velocity over real TCP, malformed/missing velocity
   rejection, and JSON/embedded-schema MCAP readback. Extend pipeline readback.
4. Document the field, plotting paths and coordinated publisher/consumer update.

## Interfaces and scope

No new topic, endpoint, dependency, physics calculation or angular velocity field.
Position and velocity use the same immutable physics snapshot and simulation time.
Native validation and the JSON schema require three numeric components bounded
by +/-1e9, matching existing vector boundary checks. Runtime JSON handling rejects
non-finite values. Rebuild/restart the renderer and restart publisher/recorder
with the updated shared schema together; old pose v1 streams are rejected.
Old MCAP files retain their embedded schema and remain readable.

## Verification

Passed:
- Shared schemas: 55 tests.
- Physics ZMQ publisher and simulation control: 20 tests, including signed nonzero
  velocity on active and held packets over real TCP.
- Recorder: 57 tests, including exact velocity/embedded-schema MCAP readback and
  malformed/non-finite velocity rejection.
- Native renderer: CMake debug configure/build and 19 real-socket adapter tests,
  including missing/old-version/malformed velocity rejection and recovery.
- Real Godot/Bullet/MCAP check: three finalized recordings with finite pose-v2
  velocity, 61/62/62 camera images and 63/63/63 poses. Ran against a temporary
  copied Godot project and isolated random TCP ports to preserve the user's
  existing tools. Report: `/tmp/bullet-velocity-recording-check/report.json`.

Full physics suite and PlotJuggler GUI plotting were not run; no physics model or
layout changed. Existing running tools need restarting to load the new contract.

## Status

Complete; committed. Implementation commit: `51e37e60078dc27a0996f3830acf80721b739a2b`.

Coordinated sibling implementation commits:

- `bullet_msgs`: `55b624df653c5b6d80c1842e9e363cb9bdbb180f`
- `bullet_render`: `0654458cbaa5c3468c66ffb192a53bf05d482ada`
- `mcap_bagger`: `817acebb9a05de7f9f4a775fd5800a7611c3b427`
