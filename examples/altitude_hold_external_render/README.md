# Altitude hold with an external renderer

One command starts a headless PyBullet simulation and the packaged Godot viewer,
waits for a real camera image, then takes off automatically. The internal altitude
controller climbs to **3 m**, holds for **10 simulated seconds after settling**,
lands, disarms and exits. Godot displays the drone and camera; it does not control
the mission. No Godot editor, app, recorder or Betaflight process is launched.

## Build and run

From the repository root:

```bash
uv run --locked python external_render/build.py
uv run --locked --extra render python -m examples.altitude_hold_external_render.main
```

Alternatively, choose **Tasks: Run Task → Bullet: Run example → Altitude hold
(external renderer)**. Build the package first. A working desktop/OpenGL driver
is required. Physics is always paced at 240 Hz; the camera defaults to 30 Hz.

For a relocated package or alternative port profile:

```bash
uv run --locked --extra render python -m examples.altitude_hold_external_render.main \
  --renderer /path/to/linux-x86_64/render \
  --render-config /path/to/linux-x86_64/configs/physics.json
```

The camera profile points to its adjacent `transport.json`. Stop other processes
using TCP 5555, 5556 or 5557, or choose unused ports in that profile. No external
Start button is needed. Closing the renderer before completion ends the example
with an error; Ctrl+C closes both processes. Missing package, unavailable ports,
startup timeout or runtime failure also releases physics, sockets and the child.

## Communication

```mermaid
flowchart LR
    Owner["Python example"] -->|Owns the automatic flight| Physics["Mission, internal controller and PyBullet, 240 Hz"]
    Physics -->|Observations| Publisher["Existing RenderPublisher"]
    Publisher -->|"ZMQ TCP 5555: config, profile, stamped poses"| Renderer["Exported Godot executable"]
    Renderer -->|"ZMQ TCP 5556: image descriptor"| Reader["Python startup image check"]
    Renderer -->|RGB8 pixels| SHM["Local POSIX shared memory"]
    SHM -->|Validate and copy pixels| Reader
    Reader -.->|First image ready, start mission clock| Physics
    Owner -.->|Launch and close| Renderer
```

| Path | Data and ownership |
| --- | --- |
| Simulation → renderer, TCP 5555 | Camera config/profile and selected poses, including position, quaternion, world velocity and source identity/time. The physics publisher binds; Godot subscribes. |
| Renderer → Python, TCP 5556 | A `camera/front/image` descriptor names the shared-memory slot, image shape, sequence and source stamp. Godot binds; the startup checker subscribes. |
| Renderer → shared memory | Actual RGB8 pixels remain local. The example verifies the first frame before starting its mission clock. The renderer continues updating its preview afterward. |
| Adapter control endpoint, TCP 5557 | Existing native adapter still binds this port and publishes heartbeats. The example does not subscribe or wait for Start/Stop/Reset. |
| Python → renderer process | Launch the exported package, watch for early exit and close the owned child on every exit path. |

Pose/image publication remains asynchronous and bounded; physics does not wait
for every rendered frame. Image source stamps identify their original physics
observation. Startup waits up to 10 host seconds for a matching stream/episode
image with readable pixels. Simulation time remains at zero during that wait.

## Mission and feedback

```mermaid
stateDiagram-v2
    [*] --> SETTLE: First camera image is ready
    SETTLE --> TAKEOFF: 0.5 s elapsed
    TAKEOFF --> HOLD: Near 3 m and slow continuously for 1 s
    HOLD --> LAND: 10 s elapsed
    LAND --> DISARMED: Grounded and slow continuously for 0.5 s
    DISARMED --> COMPLETE: 1 s elapsed with motors off
    COMPLETE --> [*]: Close physics and viewer
```

`mission.py` and `demo.py` are local to this example. No implementation is imported
from another example. Altitude P control produces vertical-speed targets, the
vertical-speed PI produces acceleration, and gravity compensation produces
collective thrust. The internal attitude/rate controller and motor mixer then
drive the simulated rotors. Feedback is ground-truth Z and vertical speed, with
no XY position hold or tilt compensation.

Takeoff/hold target Z is 3 m, with a speed-target limit of 1 m/s. Landing targets
0 m, with a speed-target limit of 0.5 m/s. Hover tolerance is 0.05 m and 0.1 m/s;
touchdown tolerance is 0.02 m and 0.05 m/s. A broken settling condition clears its
dwell timer. Default mission timeout is 45 simulated seconds. The viewer owns no
rigid-body flight dynamics and never applies forces to PyBullet.

See the [package README](../../external_render/README.md) for distribution contents,
build-time requirements and platform limits.

The process/transport adapter lives in `external_render/renderer.py` and is also
used by the automatic position-square CLI. Flight mission logic remains in each
example.
