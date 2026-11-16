# Three mutation-only request topics

SimulationService in the wind example binds request_endpoint, default tcp://127.0.0.1:5560. The renderer connects a REQ socket. These are three application-level topics on one endpoint, not PUB/SUB subscriptions. Every call carries a dataclass-exported dictionary with exactly topic, drone_id and payload. REQ/REP pairs each call with one reply, without a public request ID or an extra transport envelope.

| Topic | Payload | Restriction |
| --- | --- | --- |
| spawn | position_m: XYZ metres in Bullet world coordinates, Z up | Stopped-only, immediately initialize a fresh stopped run |
| wind | enabled, behavior, velocity_world_m_s, gust_stddev_m_s, correlation_time_s, seed | Running or stopped |
| command | action: start, stop or reset | Reset stopped-only, Start requires a prepared run |

```json
{"topic":"spawn","drone_id":"drone_1","payload":{"position_m":[0,0,3]}}
```

Replies are exactly {"success":true} or {"success":false,"reason":"spawn_requires_stopped"}. They carry no IDs, lifecycle, settings or applied values. All requests/payloads/replies have strict frozen dataclass contracts in bullet_simulation_protocol.requests, re-exported through bullet_msgs.requests. Stop ends the run, Reset retains spawn/wind and prepares another run, and completion keeps the process available. Spawn sets only position, resets attitude/velocities, retains wind and uses the simulation-configured climb height.

request_io has request_timeout_s and request_max_queue_age_s, both defaulting to 1 second. The latter uses server receive time, with no client timestamp. Requests waiting too long are rejected before mutation. An already started operation can complete after the client timeout. Repeated calls are independent, without duplicate detection or automatic retry. A timeout displays Outcome unknown.

The helper uses internal tokens to match queued application results. These tokens never appear on the wire. Its worker owns sockets and serialization. The application drains at most one operation per loop boundary. Spawn and reset initialization happen while stopped. A REP socket serializes exchanges; callers never wait for network completion inside physics stepping.

The renderer queues drafts using the drone ID bound by the minimal pose message. It sends no getters or periodic polls, and consumes only per-call replies. Future simulation-state and wind subscription topics, lifecycle display and live-wind visualization are deferred. Editors are request clients, not authoritative state displays. This is a breaking migration of both peers. Only the wind example starts the service in this milestone.

Manual checks: each topic succeeds with valid inputs, wrong drone and malformed payloads fail, spawn/reset fail while running, wind succeeds in both states, Stop prevents Start until Reset, repeated Reset calls are independent, queued expiry produces failure, and timeouts never trigger a resend. Tests and builds have not been run.
