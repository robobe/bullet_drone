# Wind request/reply protocol

Wind uses the existing thread-owned ZmqIo REQ/REP implementation. The wind-control
example binds REP at `wind_endpoint`; Godot connects REQ. Other examples omit
this endpoint and keep wind controls unavailable. No controller lease is required.

## Dataclasses and wire format

`bullet_simulation_protocol.wind` defines frozen WindSettingsMessage,
GetWindRequest, SetWindRequest, WindReadback and WindReply. Fields are validated
at construction and dictionary input, with plain dictionary/array export. JSON
is the cross-language encoding; there are no wind JSON Schema files.

Transport request: `{"request_id":"unique-id","payload":{...}}`.
Transport reply: `{"request_id":"unique-id","status":"ok","payload":{...}}`.
Malformed transport requests return transport errors; valid transport envelopes
with invalid wind payloads return rejected WindReply values.

| Payload | Fields / meaning |
| --- | --- |
| GetWindRequest | `action: get_wind` |
| SetWindRequest | `action: set_wind`, `physics_id`, `episode_id`, `issued_ms`, `settings` |
| WindReply | `status`: current/applied/rejected, `reason`, `state`: WindReadback |
| WindReadback | `physics_id`, `episode_id`, `step_index`, `simulation_time_s`, `settings`, `velocity_world_m_s` |
| WindSettingsMessage | `enabled`, `behavior`: constant/gusts, `velocity_world_m_s`, `gust_stddev_m_s`, `correlation_time_s`, `seed` |

XYZ velocity is world-frame m/s with Z up and indicates the direction of air
travel. Gust deviations are nonnegative m/s; correlation time is positive seconds.
Seed is an integer. Numbers must be finite. `issued_ms` uses Linux monotonic
milliseconds; requests must match the current simulation and episode. Replies
include the snapshot after an accepted change, without advancing physics.
`step_index` identifies the latest physics observation; `simulation_time_s` is
the wind snapshot evaluation time, which may precede that observation by one step.

## Configuration

The optional `wind_io` object is in the transport profile. If provided, include
all three fields below; omission uses the defaults in the service and renderer.

| Key | Default | Meaning |
| --- | --- | --- |
| `wind_endpoint` | TCP 127.0.0.1:5560 in wind profile only | Simulation REP binds, renderer REQ connects; must differ from other endpoints |
| `wind_io.poll_hz` | 10 | Positive readback polling rate, wall time |
| `wind_io.request_timeout_s` | 1 | Positive client timeout and readback freshness limit |
| `wind_io.request_max_age_s` | 1 | Positive maximum age for set requests |

Apply has priority over the next query; only one request is outstanding. The
native socket is recreated after an exchange/timeout. Slow or absent services do
not block rendering. A timeout is unconfirmed, not proof of rejection: subsequent
queries show actual settings before the user chooses whether to retry manually.

The application drains at most one request before each physics step. Reads use
the immutable current WindSnapshot and do not step physics or consume RNG values.
Rejected requests do not mutate wind. Existing set_wind behavior preserves gust
history for mean/deviation/correlation edits, and resets it for seed/mode or
activation changes as documented in [aerodynamics](aerodynamics.md#runtime-wind-updates).

## Migration

`simulation/wind_request` and `simulation/wind_state` are removed, together with
their validators, schemas and RenderPublisher wind scheduling. The former
`publish_rates_hz.wind` option is removed; use `wind_io.poll_hz` for readback.
The Godot windsock uses replies, not a subscription. The Tk editor is removed.
External consumers of the former topics must migrate separately. CameraInfo,
pose/state publication, SHM images, lifecycle and Betaflight remain unchanged.

- [Dataclasses](../../libs/bullet_simulation_protocol/src/bullet_simulation_protocol/wind.py)
- [Application service](../../examples/wind_control/service.py)
- [Example and diagram](../../examples/wind_control/README.md)
