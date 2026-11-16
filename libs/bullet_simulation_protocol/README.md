# Bullet simulation protocol

`bullet-simulation-protocol` provides simulation JSON contracts, validation,
packaged schemas and local transport-profile loading. Import it as
`bullet_simulation_protocol`. Message labels below describe wire payloads, not
Python dataclass names; current callers use dictionaries and validators.

## Topics and messages

This table lists retained contracts, including legacy contracts for external
consumers. Current simulation examples publish render config/profile/pose,
simulation state only. There is no local lifecycle,
spawn, collision or app RC receiver.

| Topic | Message / purpose | Direction | Default TCP port | JSON schema |
| --- | --- | --- | --- | --- |
| `camera/front/camera_info` | Camera dimensions, field of view and capture rate | Simulation → renderer | 5555 | Validated CameraInfo dataclasses, no JSON Schema |
| `render/drone_1/profile` | Profile name, backend and drone model | Simulation → renderer | 5555 | No packaged schema; built by RenderPublisher |
| `render/drone_1/pose` | Position, attitude quaternion, velocity and sample identity | Simulation → renderer | 5555 | [pose.json](src/bullet_simulation_protocol/schemas/pose.json) |
| `simulation/command` | Start, stop or reset request | Renderer/controller → simulation | 5557 | [command.json](src/bullet_simulation_protocol/schemas/command.json) |
| `simulation/heartbeat` | Controller liveness, command payload with heartbeat action | Renderer/controller → simulation | 5557 | [command.json](src/bullet_simulation_protocol/schemas/command.json) |
| `simulation/state` | Lifecycle, episode and reason | Simulation → clients | 5555 | [state.json](src/bullet_simulation_protocol/schemas/state.json) |
| `flight/drone_1/feedback` | Sensor feedback, attitude, applied RC and armed/mode state | Simulation → external app | 5555 | [flight_feedback.json](src/bullet_simulation_protocol/schemas/flight_feedback.json) |
| `simulation/spawn_request` | Initial position and takeoff-height request | Renderer/controller → simulation | 5557 | [spawn_request.json](src/bullet_simulation_protocol/schemas/spawn_request.json) |
| `simulation/spawn_state` | Applied spawn settings and request outcome | Simulation → renderer | 5555 | [spawn_state.json](src/bullet_simulation_protocol/schemas/spawn_state.json) |
| `render/drone_1/collision` | Collision proposal tied to a known simulation sample | Renderer → simulation | 5557 | [collision.json](src/bullet_simulation_protocol/schemas/collision.json) |

`flight.py` also retains the `MISSION_TOPIC` constant (`app/drone_1/mission`).
That app-owned message has no schema or validator in this library and is not
published by bullet_drone's simulation publisher.

## Transport and conventions

Simulation messages use two ZeroMQ multipart frames: UTF-8 topic, then UTF-8 JSON.
Default endpoints use `tcp://127.0.0.1:<port>`. Simulation binds its publisher on
5555. The renderer reserves control port 5557, but the local command receiver
has been removed. Consumers connect and subscribe. Endpoints are configurable;
not every example enables every message. `drone_1` is the existing fixed wire slot.

World position is in metres, velocity in metres per second, simulation time in
seconds, and attitude is a body-to-world quaternion in XYZW order. Source stamps
associate samples with physics, stream, episode, step and monotonic measurement
time. Consult the linked schemas and validators for supported versions and fields.

The wind-control example accepts dataclass get/set requests through REQ/REP.

Camera image descriptors on port 5556 and SHM pixels belong to the separate
[image-transport library](../README.md), not this simulation-message library.
Godot consumes the wire format without importing the Python package.

## Schema access

```python
import json
from bullet_simulation_protocol import schema_bytes

pose_schema = json.loads(schema_bytes('pose.json'))
```

Schemas are included in the wheel. Runtime validation uses the focused Python
validators; `jsonschema` is used by development tests.

The RC command contract and `flight_command_endpoint` are removed. Other retained
contracts support existing consumers, but are not all published by current examples.

Scenario end messages and embedded lifecycle terminal results are removed.
Simulation lifecycle state uses schema version 1. Collision proposals remain a
retained contract, without a local receiver.

The former flight configuration topic/schema/validator are removed. Renderer
configuration and flight feedback contracts remain. External consumers importing
the removed flight-config contract must migrate separately.

Camera info uses validated frozen dataclasses with dictionary export; other
contracts retain their schemas. See [the migration guide](../../docs/design/camera-info-zmq.md).

Wind now uses request/reply only in the wind example. See [wind protocol](../../docs/design/wind-request-reply.md).
