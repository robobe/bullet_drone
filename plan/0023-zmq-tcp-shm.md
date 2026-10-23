# ZMQ TCP and shared-memory images

Status: Complete; committed. Implementation commit: `2ec58b5318df28a2de180e0574358b4ae39cf2fd`.

## Goal and defaults
Migrate bullet_drone, bullet_render and mcap_bagger from IPC to loopback TCP.
Physics binds 127.0.0.1:5555; Godot and MCAP subscribe there. Godot binds
127.0.0.1:5556 for SHM descriptors; MCAP and video_viewer subscribe there.
Images use three nonblocking flock-protected POSIX SHM slots. Delivery is best
effort, with explicit skips. Keep physics scheduling and telemetry JSON unchanged.

## Steps and interfaces
Create sibling uv packages bullet_msgs (shared schemas, descriptor/header contract,
C++ definitions/writer, Python reader/demo writer) and video_viewer (one live topic,
OpenCV FPS/status overlay, reconnect and clean exit). Move renderer schemas to
bullet_msgs. Replace recorder raw_image input with shm_image, copy pixels before
queueing, retain Foxglove MCAP formats and arrival timestamps. Migrate demo,
fixtures, tooling, documentation and Bullet workspace/PlotJuggler tasks. Preserve
existing user edits. Use editable sibling dependencies and locked uv environments.

Image notifications are a single message: topic NUL, LE uint32 JSON length,
JSON descriptor (<=1024 bytes). Header is 128 bytes, <8s6I4Q64s. Slots are
0600, session-specific, validated by readers, unlinked only by their owner.
Malformed data fails recording; unavailable/stale/incomplete slots are counted skips.

## Verification
Run Python suites, real TCP/SHM fault and cleanup tests, native debug build,
headless transforms/FOV, actual GPU rendering and full physics/MCAP/two-viewer
smoke checks. Inspect recorded pixels and capture viewer screenshot. No performance
tuning, playback, remote SHM, commits or pushes. Record actual results below.

## Outcome
Implemented across bullet_drone, bullet_render, mcap_bagger, and new sibling
projects bullet_msgs/video_viewer. The Bullet workspace adds the two projects
and viewer task; PlotJuggler uses TCP 127.0.0.1:5555. Recorder configs now use
shm_image and packaged bullet_msgs:pose.json. Native adapter methods are unchanged;
statistics include session, busy slots and publication drops.

Verification:
- bullet_msgs: 34 tests passed, including killed readers/publishers, interrupted
  writes, shared locks, rotation, malformed descriptors, allocation failure cleanup,
  layout validation, restart, permissions and private copies.
- bullet_render: 16 real native/TCP/SHM integration tests passed.
- mcap_bagger: 39 tests passed.
- bullet_drone: all 579 tests passed; migrated stale UDP PlotJuggler assertions
  to the existing ZMQ layout and the agreed TCP profile.
- video_viewer: real TCP/SHM test passed.
- CMake debug configure/build, headless transforms/FOV and actual NVIDIA
  RGB/orientation/FOV/UI-exclusion checks passed. Native dependency pins unchanged.
- bullet_msgs wheel built and inspected: all three schemas included.
- Full fault run with physics, GPU renderer, MCAP and two actual viewer processes:
  late recorder, physics pause, stalled renderer and renderer restart passed.
  Independent MCAP readback verified 336 images and 805 poses without recorder
  errors; delivered rate was 16.32 FPS over 20.04 seconds, not a 30-FPS guarantee.
- Final smoke after cleanup/validation fixes: 131 images, 373 poses, 15.96 FPS
  over 8.02 seconds, no recorder errors or SHM reader skips. Report/camera PNG,
  actual viewer screenshot and process logs: /tmp/bullet-tcp-shm-final/.
  Earlier fault report: /tmp/bullet-tcp-shm-verified/report.json.

Godot SIGINT is abrupt on this installation. Normal adapter close unlinks owned
slots (tested); test runners reap only their stopped publishers' exact logged
sessions after signals. Abrupt production termination may require manual cleanup.
No performance tuning or interactive PlotJuggler connection was performed.
Implementation committed; no pushes were made.

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
