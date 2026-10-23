# ZeroMQ Godot performance experiment

Status: Complete; committed. Implementation commit: `f9f4cc95c1b38a06b3d5d0c90d1f8b2755abfa1a`.

Goal: replace Zenoh across physics, native Godot and MCAP with local ZeroMQ PUB/SUB IPC; compare three 60-second NVIDIA runs against preserved Zenoh commits.

Steps: rename optional example-side publisher to zmq_render; preserve physics scheduling, one-entry queue and cleanup; use a shared JSON transport profile and two-part topic/payload messages. Republish startup configuration every second. Replace optional render dependency with PyZMQ. Update tests, examples and design documentation.

Interfaces: render profile uses transport_config, camera and fps. Transport JSON has telemetry_endpoint and image_endpoint. Physics binds telemetry; Godot and recorder connect. Godot binds images; consumers connect. Linux abstract IPC endpoints by default.

Verification: existing physics suite plus real transport startup, restart, malformed messages and cleanup checks; camera/MCAP integration, slow and late subscribers; matched NVIDIA benchmarks and independent MCAP readback.

Defaults: 640x480 RGB8, 90-degree horizontal FOV, 30 Hz capture, 240 Hz physics unchanged. PlotJuggler UDP unchanged. IPC socket images replace SHM; no custom shared buffers, runtime transport selection or automatic pushes. Commits were subsequently requested by the user.

Verification outcome: 579 physics tests passed with the render extra; 11 native adapter tests and 35 recorder tests passed. Real-render projection/pixel checks and pipeline late-start/pause/stall/restart checks passed. Six matched 60-second NVIDIA recordings passed independent MCAP readback. Median FPS: Zenoh 28.68, ZeroMQ IPC 28.96; no meaningful frame-rate gain established. Publication call median fell from 0.736 ms to 0.143 ms. Detailed results and baseline hashes are in sibling bullet_render/BENCHMARK.md and benchmark-results.json.

VS Code task configuration and dependencies validated; interactive VS Code and PlotJuggler UI were not exercised. Implementation committed at the user’s request; not pushed.
