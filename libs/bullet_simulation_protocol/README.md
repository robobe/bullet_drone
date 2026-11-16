# Simulation protocol

Validated Python dataclasses define process-neutral messages and export dictionaries. Non-Python clients exchange JSON.

| Interface | Default endpoint | Purpose |
| --- | --- | --- |
| render/drone_1/pose | PUB/SUB TCP 5555 | Minimal camera pose, configured drone ID and simulation-step header |
| camera/front/camera_info | PUB/SUB TCP 5555 | Camera calibration and dimensions |
| render/drone_1/profile | PUB/SUB TCP 5555 | Renderer profile display |
| flight/drone_1/feedback | PUB/SUB TCP 5555 | Complete ground truth and sensor telemetry |
| spawn, wind, command | REQ/REP TCP 5560 | Mutation-only control requests |

The request envelope contains exactly topic, drone_id and payload. Spawn payload is position_m only; wind payload includes all wind settings; command payload contains action start/stop/reset. Replies contain success:true, or success:false plus reason. There are no public request IDs, schema versions, client timestamps, getters, readbacks or automatic retries. Queue tokens remain internal to ZmqIo. request_endpoint and request_io configure the shared service.

Import SpawnRequest, SpawnPayload, WindRequest, CommandRequest, CommandPayload and OperationReply from bullet_simulation_protocol.requests. Their from_dict methods validate untrusted dictionaries, and to_dict exports JSON-ready values. The primary wind module retains WindSettingsMessage. Legacy command, state, pose, spawn and wind JSON Schemas are removed; the retained collision schema is separate.

See [three request topics](../../docs/design/three-request-topics.md), [minimal pose](../../docs/design/minimal-camera-pose.md), [complete feedback](../../docs/design/simulation-feedback.md), and [simulation I/O](../../docs/simulation/io.md). Future renderer state/wind subscriptions are deferred. Only the wind example starts the shared request service in this milestone.
