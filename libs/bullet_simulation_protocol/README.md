# Bullet simulation protocol

`bullet-simulation-protocol` provides simulation JSON contracts, validation,
packaged schemas and local transport-profile loading. Import it as
`bullet_simulation_protocol`. Message labels below describe wire payloads, not
Python dataclass names; current callers use dictionaries and validators.

## Topics and messages

| Topic | Message / purpose | Direction | Default TCP port | JSON schema |
| --- | --- | --- | --- | --- |
| `render/drone_1/config` | Camera dimensions, field of view and capture rate | Simulation → renderer | 5555 | [config.json](src/bullet_simulation_protocol/schemas/config.json) |
| `render/drone_1/profile` | Profile name, backend and drone model | Simulation → renderer | 5555 | No packaged schema; built by RenderPublisher |
| `render/drone_1/pose` | Position, attitude quaternion, velocity and sample identity | Simulation → renderer | 5555 | [pose.json](src/bullet_simulation_protocol/schemas/pose.json) |
| `simulation/command` | Start, stop or reset request | Renderer/controller → simulation | 5557 | [command.json](src/bullet_simulation_protocol/schemas/command.json) |
| `simulation/heartbeat` | Controller liveness, command payload with heartbeat action | Renderer/controller → simulation | 5557 | [command.json](src/bullet_simulation_protocol/schemas/command.json) |
| `simulation/state` | Lifecycle, episode, reason and optional terminal result | Simulation → clients | 5555 | [state.json](src/bullet_simulation_protocol/schemas/state.json) |
| `flight/drone_1/rc` | 18 RC channels, app identity, sequence and expiry | External app → simulation | 5559 | [flight_rc.json](src/bullet_simulation_protocol/schemas/flight_rc.json) |
| `flight/drone_1/config` | Vehicle parameters, RC mapping, camera and timing | Simulation → external app | 5555 | [flight_config.json](src/bullet_simulation_protocol/schemas/flight_config.json) |
| `flight/drone_1/feedback` | Sensor feedback, attitude, applied RC and armed/mode state | Simulation → external app | 5555 | [flight_feedback.json](src/bullet_simulation_protocol/schemas/flight_feedback.json) |
| `simulation/spawn_request` | Initial position and takeoff-height request | Renderer/controller → simulation | 5557 | [spawn_request.json](src/bullet_simulation_protocol/schemas/spawn_request.json) |
| `simulation/spawn_state` | Applied spawn settings and request outcome | Simulation → renderer | 5555 | [spawn_state.json](src/bullet_simulation_protocol/schemas/spawn_state.json) |
| `simulation/wind_request` | Enable, mean wind, gust settings and seed | Renderer/controller → simulation | 5557 | [wind_request.json](src/bullet_simulation_protocol/schemas/wind_request.json) |
| `simulation/wind_state` | Applied settings, current world wind and request outcome | Simulation → renderer | 5555 | [wind_state.json](src/bullet_simulation_protocol/schemas/wind_state.json) |
| `render/drone_1/collision` | Collision proposal tied to a known simulation sample | Renderer → simulation | 5557 | [collision.json](src/bullet_simulation_protocol/schemas/collision.json) |
| `scenario/ended` | Confirmed terminal outcome and final sample | Simulation → clients | 5555 | [scenario_end.json](src/bullet_simulation_protocol/schemas/scenario_end.json) |

`flight.py` also retains the `MISSION_TOPIC` constant (`app/drone_1/mission`).
That app-owned message has no schema or validator in this library and is not
published by bullet_drone's simulation publisher.

## Transport and conventions

Simulation messages use two ZeroMQ multipart frames: UTF-8 topic, then UTF-8 JSON.
Default endpoints use `tcp://127.0.0.1:<port>`. Simulation binds its publisher on
5555; Godot binds the control publisher on 5557; the external app binds its RC
publisher on 5559. Consumers connect and subscribe. Endpoints are configurable;
not every example enables every message. `drone_1` is the existing fixed wire slot.

World position is in metres, velocity in metres per second, simulation time in
seconds, and attitude is a body-to-world quaternion in XYZW order. Source stamps
associate samples with physics, stream, episode, step and monotonic measurement
time. Consult the linked schemas and validators for supported versions and fields.

The automatic wind-control example applies GUI settings locally, then publishes
`simulation/wind_state`; it does not send `simulation/wind_request`.

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
