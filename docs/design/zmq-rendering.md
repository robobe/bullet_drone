# Optional Godot camera publication

The automatic Angle and Betaflight Acro square CLIs accept `--render-config` and
default to `external_render/configs/physics.json`. Standalone internal altitude
hold has no Godot adapter. Install with `uv sync --extra render`. Each rendered
application owns its publisher/viewer lifecycle; imports start no I/O. The
library remains transport-independent. Existing UDP and ZeroMQ render adapters
remain available.

RenderPublisher selects observations at camera FPS in simulation time and puts
immutable resolved poses into a one-entry queue without blocking. A worker owns ZeroMQ,
JSON encoding and TCP publication. Full queues replace old states and increment a
counter. Episode changes restart the schedule. Stream sequence remains monotonic.
The example paces headless execution when this option is used. All workers close
on completion, interruption or failure. A worker failure is logged without stopping
an already running physics loop; startup failure prevents starting the example.

The JSON startup profile requires fps, camera (width_px, height_px, fov_x_deg), and
a relative transport_config path to JSON containing telemetry_endpoint and image_endpoint. Defaults in the supplied profile are 640x480, 90-degree
horizontal FOV, 30 Hz; the camera is centered and forward-facing. One drone_1 stream
is supported. Settings are immutable during a run. Startup config is republished
once a wall-clock second for late subscribers.

Topics: render/drone_1/config and render/drone_1/pose. Pose v2 messages contain position,
body-to-world XYZW orientation, world ground-truth `linear_velocity_m_s` ([vx, vy, vz]
in m/s, Z up), drone/episode/stream identity, step, simulation time,
sequence and publisher_drops. Shared schemas are packaged by sibling bullet_msgs; renderer/recorder
profiles remain in sibling bullet_render. No physics dependencies leak into the native adapter.

Godot requests are latest-only, never acknowledged back to physics. The existing
transport profile key `telemetry_endpoint` carries renderer pose/config and
application feedback topics; the separate `analysis/physics/*` sample stream has
been removed from Bullet Drone. Godot publishes image descriptors on loopback TCP
port 5556; image pixels use three nonblocking, flock-protected POSIX SHM slots
from bullet_msgs. Scenery is visual only and does not alter Bullet contacts. See
the sibling `bullet_render` README for its own viewer and consumer workflows.

The image wire format replaces the old socket-pixel format. See ../bullet_msgs/README.md
from the projects directory for the shared protocol. The video_viewer sibling uv
project shows one live topic. Capture cadence and the config schema remain unchanged.
Pose v2 requires velocity from the same immutable ground-truth state for both
active and held packets. Rebuild the native renderer and restart publisher and
recorder together; pose v1 is rejected by the updated strict consumers.

## Automatic square viewer

The angle-square CLI owns the exported viewer through `external_render.renderer.ExternalRenderer`,
shared with the external-render altitude example. It waits for the first matching
episode image before mission timing starts, publishes running state, and flies
automatically with headless PyBullet and wall-time pacing. It exits after landing;
viewer loss fails the run and closes all resources. `--headless` skips rendering
and transport entirely. `--renderer` selects the exported executable and
`--render-config` defaults to the packaged `external_render/configs/physics.json`.
Programmatic `run_demo(control=...)` remains available.

The Betaflight Acro-square CLI has its own renderer adapter rather than importing
another example. It uses the same packaged viewer/readiness protocol, advertises
`backend=betaflight`, starts automatically, and closes the viewer with its owned
SITL session. Its `--headless` path starts no rendering or ZeroMQ resources.

## Command-driven external-RC flow

A main-thread SimulationControl SUB receives exact simulation/command and
simulation/heartbeat messages from Godot PUB on TCP 5557. Physics/mission reset and
stepping remain inside each example's loop; no receiver thread mutates physics.
Start resets all episode state and pacing. Stop freezes the final observation;
Reset restores initial state at time zero without stepping. Completion/errors
keep the process available. Godot loss for three seconds ends the active run.
Malformed, stale, duplicate and retired-controller messages are ignored.

Commands have no acknowledgments or barrier. Physics publishes simulation/state
on port 5555 at transitions and every second. Godot shows this actual state and
requires a fresh heartbeat relationship before enabling controls. RenderPublisher
holds immutable observations and repeats stopped poses at 1 Hz for late viewers,
without advancing simulation time; active capture scheduling remains unchanged.

## Sensor-derived camera pose

The external-RC example accepts `--config configs/basic.yaml` for physics YAML.
Set `pose_sources.altitude: barometer` and `pose_sources.attitude: imu` to render
measured altitude and complementary-filter attitude. The same attitude is sent
to the navigation app. Ground-truth defaults, true XY/velocity and all capture
source stamps are preserved. Held packets use the cached resolved pose.

Noise is generated only by the physics sensors, with per-sample standard
deviations, fixed biases and seeds in YAML. The filter runs at physics frequency
and restarts on reset/spawn. A required disabled sensor is rejected, without
ground-truth fallback. See [sensor pose configuration](sensors.md#sensor-pose-selection-and-attitude-estimation).

The JSON render profile continues to configure transport, capture FPS and camera.
The older standalone render YAML source selectors are unchanged.
