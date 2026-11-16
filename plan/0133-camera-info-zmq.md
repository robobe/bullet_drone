# CameraInfo and ZeroMQ I/O

## Goal and implementation
Replace render/drone_1/config with camera/front/camera_info from validated frozen ROS-shaped dataclasses and dictionary export. Centralize Python ZeroMQ sockets in bullet_simulation_protocol.transport in a dedicated worker with bounded queues, PUB/SUB and optional REQ/REP. Migrate Godot and renderer adapters; preserve physics ownership and Betaflight.

## Interfaces and defaults
Camera-info envelope contains schema_version, stream_id, capture_fps_hz and camera_info. Static optical-frame header, ideal pinhole matrices, zero distortion. Camera-info publication defaults 1 Hz; configured pose fps stays 30. Other periodic publication rates configurable. No camera query or lifecycle endpoint enabled. Other schemas unchanged.

## Verification
Passed: combined project/protocol suite 700 passed, 8 skipped; native Godot adapter 25 passed; post-review focused camera/transport/publisher/renderer checks 42 passed; malformed deep-JSON transport checks 5 passed. Full renderer package rebuilt. Real viewer readiness received matching 640x480 source-stamped image (921600 RGB bytes). Edited Mermaid diagrams parsed/rendered; local links and whitespace checked.

Transport includes real TCP PUB/SUB exact-topic checks, no-consumer bounded snapshot submission, REQ/REP timeout recovery, malformed requests and bind-failure cleanup. Camera tests cover pinhole matrices, dictionary round-trip, unsupported calibration, invalid rates and independent 10 Hz camera-info/30 Hz capture settings.

No full flight mission or hard real-time guarantee. Existing external consumers of the old topic must migrate.

## Scope and limits
No ROS dependency or unrelated schema migration. Python threads avoid network waits in physics but do not guarantee hard real-time isolation. Old wire consumers require migration.

## Status
Implemented and verified; uncommitted.

## Implementation commit
Pending.
