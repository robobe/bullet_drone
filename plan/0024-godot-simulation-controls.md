# Godot simulation controls over ZMQ PUB/SUB

Status: Complete; committed. Implementation commit: `2ec58b5318df28a2de180e0574358b4ae39cf2fd`.

## Goal and scope
Godot broadcasts Start/Stop/Reset without acknowledgments or participant barriers.
All three example CLIs start stopped and stay alive between runs. Start creates
fresh physics/sensor/controller/mission state; Stop ends the episode at its final
pose; Reset restores initial state and waits. Mission completion stays stopped.
Godot heartbeat every 500 ms, timeout 3 seconds stops physics. Commands older than
one second, duplicate/out-of-order messages and old controller sessions are rejected.
Godot shows actual physics state, never optimistic command completion.

## Implementation and interfaces
Add control_endpoint TCP 127.0.0.1:5557, bound by Godot, subscribed by physics and
future consumers. Keep telemetry/state 5555 and image descriptors 5556. Add command
and state contracts/validation to bullet_msgs. Use bounded nonblocking queues and
worker-owned native sockets. Physics commands apply between steps in example-owned
loops. Preserve immediate one-shot direct run_demo calls without control adapter.
All CLIs use render extra/shared profile; add render-config to altitude/Acro.
Idle poses/state repeat at 1 Hz for late subscribers. Add bottom-left accessible
SVG play/stop/reset toolbar, real state/error/disconnected feedback, no replay on
reconnect. Cancel captures from earlier episodes. Update tasks and pipeline tooling
to send Start explicitly and maintain heartbeats. No MCAP automation or mission picker.

## Verification
Shared-schema and real TCP fanout, malformed/stale/out-of-order commands, heartbeat
expiry, fresh runs, Stop/Reset, completion and errors for all examples. Native
build/transport/headless and actual GPU toolbar checks, screenshot and full pipeline.
Preserve existing edits; no automatic commit or push.

## Outcome
Implemented the native Godot PUB command/heartbeat worker, SVG toolbar and actual
physics-state display. Shared command/state schemas, Python validation and control
publisher are in bullet_msgs. All three CLI examples initialize stopped and stay
alive across runs; direct run_demo calls retain one-shot behavior. Profiles now
require control_endpoint, and task/demo/recorder fixture profiles are migrated.
No recording lifecycle automation was added.

Verification:
- bullet_msgs: 46 tests passed, including schema and malformed-command checks.
- bullet_drone: full 593-test suite passed. After refining watchdog processing to
  consume fresh queued heartbeats before declaring controller loss, all 15 focused
  control checks passed, including the new busy-physics regression and real CLI
  Start/Stop/Reset/heartbeat-loss checks for all three examples.
- bullet_render: full 18-test suite passed, including real native command fanout
  and authoritative-state validation/expiry. Three focused native checks also
  passed after making probe cleanup retain session IDs independently of its
  bounded log buffer.
- mcap_bagger: 39 tests passed. video_viewer: its real transport test passed.
- Native debug build, unchanged-configure regression, headless transforms/FOV,
  NVIDIA RGB/orientation/FOV and UI-exclusion checks passed. Dependency pins unchanged.
- Built and inspected the bullet_msgs wheel: all five schemas and control module
  included.
- Real GPU pipeline with two viewer processes verified stopped startup, Start,
  Stop with frozen time, Reset at zero, new episode IDs, physics pause, stalled
  rendering, heartbeat-loss stop and renderer reconnect requiring a new Start.
  Independent MCAP readback verified 145 images and 402 poses without errors.
  Report/logs/camera PNG: /tmp/bullet-controls-check/ (controls_verified=true).
- Actual Godot toolbar screenshots inspected in stopped/running states:
  /tmp/bullet-controls-ui/stopped.png and running.png. Reset report and logs are
  in the same directory. Short stopped-physics CPU sample: approximately 1% of
  one core over one second, with simulation time fixed at zero.
- Removed only this task's stopped test-publisher SHM sessions. Earlier unrelated
  SHM names were retained. Normal close and crash cleanup are separately tested.

Limitations: PUB/SUB events may be missed and require another click. No per-command
acknowledgments, participant barrier, recording automation or mission picker.
Interactive keyboard/screen-reader use was not manually exercised; standard Godot
Button focus and accessibility properties are configured. Implementation committed; no pushes.

## Implementation commits

The integrated workspace state is recorded by these commits; plan metadata is
committed separately.

| Project | Implementation commit |
| --- | --- |
| bullet_drone | `2ec58b5318df28a2de180e0574358b4ae39cf2fd` |
| bullet_render | `f230bba05d9f4d6ee26a34dd76b481b23e971fc8` |
| mcap_bagger | `f19c20fcda3f9263761749ef95d1033bd11358c7` |
| bullet_msgs | `b1d7cfd72bf12efa3d6906a81109d97da7b01b3d` |
| video_viewer | `0f0d18c053c0478ae84249b3050f60e591240f12` |
| bullet | `06a4cf4d9d8c6a534b40bdee945b8226a3d3277b` |
