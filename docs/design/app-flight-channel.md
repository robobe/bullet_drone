# App flight channel

`examples.external_rc.main` owns a separate caller-paced 240 Hz `RcSimulation`
loop. Existing square missions are unchanged. It uses `SimulationControl` for
Godot lifecycle, `FlightReceiver`/`FlightLease` for local app intent and
`RenderPublisher` for 120 Hz feedback plus camera-paced poses. Runtime physics
code has no networking imports. This example is headless; Godot owns rendering.

Godot Start requires a fresh inactive/disarmed readiness packet for the current
physics/episode identity. Start resets physics, creates a fresh episode and
leases it to that app identity. Physics waits frozen up to 0.5 wall seconds for
the first active packet for the new episode. Disarmed active packets step physics
for sensor warmup. Thereafter the latest accepted full 18-channel RC frame drives
each step. The app owns ARM/Angle selection. Sequence must increase, issue time
cannot be future and validity is at most 100 ms. Wrong topics, ids, invalid,
expired and reordered packets are rejected. A lease cannot change owner while
running. Active=false withdraws the episode; expiry is checked before each step.
Stop freezes the pose; it does not land or run a hidden position controller.

`flight/drone_1/config` advertises mass, gravity, thrust authority, RC scales and
mapping, rates, camera calibration and `attitude_model=ideal` by default. With physics YAML
`pose_sources.attitude: imu`, the model is `imu_complementary` and feedback uses
the same complementary-filter estimate as rendering. Feedback includes barometer
altitude and body-to-world XYZW attitude, applied arm/mode/RC sequence, owner and
lifecycle reason. It intentionally excludes XY control feedback. Internal
stabilization remains ideal. See [sensor pose settings](sensors.md#sensor-pose-selection-and-attitude-estimation)
for `--config`, noise, initialization and yaw-drift limits.

Each reset/step stamps one source containing physics/episode/stream ids, step,
simulation time and Linux monotonic measurement microseconds. That source is
reused by feedback and pose v3; the renderer emits descriptor v2 with the same
source and capture_ns=measurement_us*1000. Integer microseconds remain exactly
representable through Godot JSON; the SHM binary header remains v1. Legacy
examples still publish pose v2 and descriptor v1 with arrival-time capture.

Telemetry ports default to 5555 (physics feedback/config/pose), 5556 (images),
5557 (Godot lifecycle), 5558 (app status/bbox), 5559 (app RC). App control runs
100 Hz; sensor feedback 120 Hz. Packet freshness uses wall monotonic time;
mission durations use simulation time. A slow UI never advances physics beyond
the RC expiry lease. Godot detects truck contact and physics confirms terminal
completion as described below; no landing or physical target body is included.

## Renderer collision termination

`ScenarioTermination` retains a bounded one-second source history. The lifecycle
SUB accepts exact `render/drone_1/collision` reports from the live controller,
matching the current episode/stream and a known source stamp. A valid proposal
is processed before stepping/RC expiry, ends the lease and freezes physics. The
first accepted end wins; stopped episodes cannot be reclassified by late reports.

The external runner emits state v2 with `terminal_result`, plus repeated
`scenario/ended` on telemetry port 5555 every 100 ms. Other examples retain state
v1. The result identifies success/target_hit, the original source/contact position
and the actual final frozen source; physics is not rewound. Reset/Start clears
history and result. Godot retries proposals on 5557 every 100 ms with a stable,
episode-specific event ID; after one second without confirmation it requests
ordinary Stop and shows failure rather than success.

The app latches COMPLETED and stops LOS/active RC; late subscribers recover the
result from repeated events/state or the target_hit stopped feedback. The
recorder queues the terminal event before closing and drains final diagnostics
for 250 ms. Geometry and impact response stay outside the physics library.

## Stopped spawn transaction

Only `examples.external_rc.main` enables `SpawnSelection` on `SimulationControl`.
Exact shared spawn requests on lifecycle 5557 require the live controller,
current physics ID, stopped current episode and <=1 s monotonic request age.
The last 128 request fingerprints/outcomes are retained. Retrying an identical
request re-advertises its outcome even after episode change, with no second
reset; changing a previously used request's payload cannot replay it. Stale,
foreign, malformed or running requests cannot alter the simulation.

The runner constructs a replacement immutable `SimulationConfig`/`DroneSettings`
with the selected initial position and initializes a candidate `RcSimulation`.
Only after successful reset does it replace and close the old stopped simulation.
Initialization failure closes the candidate, acknowledges rejection and retains
the old simulation/configuration. Successful Apply clears the app lease,
collision history/result and applied-command state, and publishes a fresh stopped
episode/pose/sensor source. No core reset signature or private state is changed.
Subsequent Start/Reset use the replacement config; normal sensor/controller reset
semantics and the app's initial barometer datum preserve relative takeoff.

`RenderPublisher.set_spawn` copies a validated latest status under its snapshot
lock. Its existing socket worker publishes changes and a 10 Hz retained status
on telemetry 5555, independent of stepping. Status always identifies the current
episode, with the applied named position and latest request outcome. Godot's
bounded native slots keep socket workers independent of Godot parsing/objects.
The renderer retries at 100 ms for 2 s; after timeout it blocks lifecycle edits
until an acknowledgement, physics restart, or a fresh authoritative status beyond
the last request/cache lifetime reconciles the result. Other examples are
unchanged. Default initial position remains (0,0,0.015) m; physical terrain is
still the flat PyBullet plane, not the renderer's visual landscape.

Spawn request/state v2 add `takeoff_height_m`, applied atomically with position.
Bounds are 0.1–100 m, default 20 m; v1 requests remain accepted with that default.
Initialization rejection retains both the position and height. Feedback v2
repeats the applied height with every current episode's source stamp. The app
latches it on the first feedback for that episode and sets goal = initial
barometer + selected height. Start/Reset keep that height; a new Apply creates a
new episode with its newly confirmed value. V1 feedback keeps legacy 20 m.
This configuration is not ground-truth position feedback for the controller.

The app's post-hover target check now enters YAW_SEARCH when no usable bbox
appears within 100 ms. It commands 15 deg/s positive yaw for at most one measured
net circle, holding altitude/level attitude. A usable candidate enters YAW_CENTER, which uses bounded proportional yaw
to put the bbox center within +/-10 image pixels of horizontal center.
YAW_SETTLE requires that band and low measured yaw speed continuously for
0.3 s, then starts LOS acquisition at the new heading. A visible off-center
target cannot start approach; drift resumes centering. Lost candidates resume the original sweep; one full circle or a 34 s
simulation-time deadline withdraws RC and freezes physics. This adds no physics
runner logic or wire changes; the app owns search state. See
[plan 0034](../../plan/0034-yaw-target-search.md).

See [plan 0035](../../plan/0035-center-target-before-los.md) for centering
defaults, diagnostics and verification.

## Live wind transaction

Only the external-RC runner enables `WindControl`. Exact `simulation/wind_request`
on the existing control endpoint (default 5557) requires the live controller,
current physics/episode identity and a request no older than one second. Apply
works while running or stopped. The runner invokes `RcSimulation.set_wind` at a
step boundary and replaces its immutable runtime config so later spawn changes
retain the applied wind. Failed updates retain the previous configuration.
The last 128 request fingerprints/outcomes make retries idempotent.

`RenderPublisher.set_wind` validates and copies a snapshot under its existing
lock; the socket worker publishes cached `simulation/wind_state` at 10 Hz on
telemetry (default 5555), including while stopped. Neither networking nor Godot
reads advance physics or gust RNG. Godot retries Apply every 100 ms for two
seconds and reconciles uncertain results against fresh authoritative readback.
Native readback expires after one second and must match current control identity.
Other examples do not subscribe to wind requests and show unavailable controls.
No new ports, disk writes, persistence or app tracking interfaces are introduced.
