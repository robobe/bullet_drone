# Bullet Drone

This project is a member of the [Bullet workspace](../README.md). All six
projects share `../.venv` and `../uv.lock`; run uv commands with the
workspace flags below to retain every project and optional dependency.

A Python quadcopter simulation using PyBullet, configurable aerodynamic forces, PID control,
and RC Angle/Acro stabilization. The bundled racer model needs no asset downloads.
The altitude-hold example takes off to 3 m, holds for 10 simulated seconds after
settling, then lands and disarms.

See the [implemented RC and internal control chain](docs/forces/drone_control.svg)
for command routing, motor mixing, physics, and ground-truth feedback.

The [force and effect implementation map](docs/forces/force-implementation-map.md)
reviews the course inventory against this code. Its
[numbered SVG tips](docs/forces/force-implementation-chain.svg) link each effect
to its owning stage and table row, marking future models separately.

## Setup and run

Use Python 3.12 and uv:

```bash
uv sync --locked --all-packages --all-extras --all-groups
```

The package is a library. Runnable examples provide their own entry points and
own their execution loops, pacing, diagnostics, and cleanup. GUI mode uses
PyBullet's built-in viewer and requires a desktop/X11 display. Headless mode
needs no display or wall-time pacing. Ctrl+C releases simulation resources.

## Altitude hold example

```bash
uv run --locked --all-packages --all-extras --all-groups python -m examples.altitude_hold.main
uv run --locked --all-packages --all-extras --all-groups python -m examples.altitude_hold.main --headless
```

The example owns its feedback loop through `RcSimulation.step()`. Each observation
supplies ground-truth altitude and vertical speed to the altitude PID. RC angle
mode keeps roll and pitch level. GUI mode is paced to wall time; headless mode
runs the same sequence faster. There is no horizontal position hold.

See [the example README](examples/altitude_hold/README.md) for gains, timing,
sequence and state-flow diagrams, and verification.

## Position square example

```bash
uv run --locked --all-packages --all-extras --all-groups python -m examples.position_square.main
uv run --locked --all-packages --all-extras --all-groups python -m examples.position_square.main --headless
```

The example owns its loop, takes off to 3 m, flies a settled 3 m square while
holding heading, holds home for 10 seconds, then lands and disarms. Separate XY
position and velocity PIDs feed the existing RC angle-mode simulation. See
[the example and diagrams](examples/position_square/README.md) and
[controller interfaces and migration](docs/design/position-control.md).

## Acro position square example

```bash
uv run --locked --all-packages --all-extras --all-groups python -m examples.position_square_acro.main
uv run --locked --all-packages --all-extras --all-groups python -m examples.position_square_acro.main --headless
```

This separate example flies the same square, holds home for 10 seconds, then
lands and disarms using Acro RC channels. Navigation converts tilt targets to
body-rate requests upstream; the simulator bypasses its internal attitude P and
uses rate PID directly. See [the Acro example and diagrams](examples/position_square_acro/README.md).

## VS Code tasks

Open **Tasks: Run Task** and choose:

| Task | Behavior |
| --- | --- |
| Run altitude hold | Takeoff, 10-second hold, landing and disarm in PyBullet GUI. |
| Run altitude hold (headless) | Same mission without a display. |
| Run position square | Settled 3 m square, home hold, landing and disarm in PyBullet GUI. |
| Run position square (headless) | Same square mission without a display. |
| Run Acro position square | Same settled square with upstream rate commands and Acro stabilization. |
| Run Acro position square (headless) | Same Acro mission without a display. |

## Library interfaces

`DroneEnvironment.reset()` and `step()` return immutable `DroneObservation`
records containing ground truth and configured sensor readings. `RcSimulation` owns RC
translation, stabilization, mixing, and physics; the caller owns its execution
loop. Call `reset()` before stepping and `close()` when finished.

Optional `drone.drone_id` in YAML defaults to `drone_1`. It is retained in
`DroneState` across resets and used in the shared simulation telemetry header.
See [drone identity and telemetry](docs/design/drone-identity-and-telemetry.md).

`AltitudeController` supplies collective thrust in newtons from world Z altitude
and vertical speed. `PositionController` supplies world XY velocity targets;
`VelocityController` supplies roll/pitch and tilt-compensated thrust from velocity
feedback and vertical force. The caller composes altitude/yaw and RC encoding.
`RcSimulation` enables IMU and barometer readings with zero noise by default.
Configure Gaussian noise, fixed biases and seeds through Python or YAML; disable
each sensor independently. Direct `DroneEnvironment` callers inject sensors
explicitly. Read `observation.sensors.imu` and `observation.sensors.barometer`.
Sensor noise never modifies ground truth or current controller feedback.
See [sensor configuration and measurement semantics](docs/design/sensors.md).

`create_flight_command_provider()` connects a caller-supplied high-level command
source to internal stabilization. Angle roll/pitch commands use radians; Acro
commands use body rates in rad/s. The RC yaw channel is a rate in both modes.
Motor targets and actual speeds use rad/s. Configuration selects the `internal`
backend by default; `betaflight` is recognized but not implemented.

Rendering adapters support GUI/headless selection and UDP pose publication through
library composition. The flight examples expose PyBullet GUI and `--headless`;
the square example now accepts the optional ZeroMQ render profile described below.
The older library render profiles are not bundled in this checkout. Physics and headless execution remain independent
of any external viewer. Callers own observation-consumer reset/consume/close.

## Aerodynamics and wind

New aerodynamic effects default to enabled, with calm ambient wind. Configure
independent drag, damping, inflow, flapping, gyroscopic, ground-effect and local
propwash settings through Python or the commented `configs/basic.yaml` loader.
Wind supports a world velocity vector and repeatable smooth gusts. Existing
example CLIs use Python defaults and do not load YAML. For exact ideal-rotor
experiments, select `ForcesSettings.ideal()` and disable ambient wind.
See [models, defaults and compatibility](docs/design/aerodynamics.md).

## Optional telemetry

The telemetry package defines ten simulation topic schemas with a shared header
(schema version, drone/episode identity, step and simulation time). Seven optional
topics can be selected with `SimulationConfig.telemetry` or the commented
`telemetry` section in `configs/basic.yaml`; ground truth, config and events are
mandatory. `selected_schemas(settings)` returns the selected topic/schema mapping.

These contracts do not start recording or change physics. The optional Angle
square example can now [stream ground truth to PlotJuggler](plotjuggler/README.md)
through a sender process configured by its own YAML outside src. General collection
and file recording remain future work; the MCAP adapter has been removed. See [schemas and the recording strategy](docs/design/telemetry.md) and the
[recorder TODO](docs/todo.md).

## Assets and verification

The racer model is adapted from MIT-licensed gym-pybullet-drones. Front propellers
are orange and rear propellers are blue. See the bundled
[asset notes and license](src/bullet_drone/assets/racer/README.md).

```bash
uv run --locked --all-packages --all-extras --all-groups pytest -q
```

Tests cover lifecycle and cleanup, configuration, gravity/contact, force and torque
directions, mixer limits, PID control, RC translation and arming, rendering adapters,
and both flight missions. Telemetry tests validate schemas, shared headers,
payloads, topic selection and import safety without a recording service.

Implementation records are in [the plan menu](plan/README.md).

## Godot / ZeroMQ camera

Install `uv sync --locked --all-packages --all-extras --all-groups`, then run the square example with
`uv run --locked --all-packages --all-extras --all-groups python -m examples.position_square.main --headless --render-config ../bullet_render/configs/physics.json`
from the physics repository root. Rendering enables wall-time pacing even without
PyBullet GUI. See [the renderer](../bullet_render/README.md) for the native Godot and recorder commands.

## Godot controls

Run this example with `uv run --locked --all-packages --all-extras --all-groups python -m examples.position_square.main --headless`.
The CLI initializes **Stopped** and waits for Godot's Start button; it no longer
runs a flight immediately. Stop ends the current episode and preserves the final
pose. Reset restores initial state at time zero and waits. Start always begins a
fresh episode. Completion leaves the process available for another run. Only one
physics example can own the shared ports at a time.

The default profile is `../bullet_render/configs/physics.json`; override it with
`--render-config`. Keep sibling bullet_msgs installed through the existing render
extra. Direct `run_demo()` calls without a control adapter remain one-shot and do
not need Godot. See [the transport/control design](docs/design/zmq-rendering.md).

## Sensor pose for Godot and app feedback

Run the external-RC example with `--config configs/basic.yaml` to load physics
sensor noise and pose settings. Set `pose_sources.altitude: barometer` and
`pose_sources.attitude: imu` to use measured render height and an IMU complementary
filter for both camera and app attitude. Existing launches retain their defaults.
See [configuration, noise and estimator limits](docs/design/sensors.md#sensor-pose-selection-and-attitude-estimation).

The external-RC runner prints a `bullet_drone loaded configuration` banner to
stdout once at startup. Its indented JSON contains the effective simulation
settings (including defaults and the headless override), flight control, vehicle,
render, transport and advertised platform configuration, plus configuration paths.
`physics_config_path: null` means the built-in defaults were used. Diagnostics
continue on stderr; the banner precedes physics initialization.

The barometer in `configs/basic.yaml` uses `filter_enabled: true` and
`filter_time_constant_s: 0.1` under `sensors.barometer`. It smooths pressure before
altitude conversion for rendering and app feedback. Set `filter_enabled: false`
for direct noisy readings. Omitted flags default to false in the library.

The IMU demo now uses an ICM-42688-P density profile with sensor filtering:
`sensors.imu.noise_model: density`, `filter_enabled: true`, acceleration cutoff
30 Hz and gyro cutoff 60 Hz. Fixed bias and residual bias drift remain zero as a
calibrated baseline. See [IMU fields, units and model limits](docs/design/sensors.md#icm-42688-p-density-based-imu-profile).
Omitted options preserve the legacy per-sample noise model and disable filtering.

### Live wind in Godot

With the external-RC/LOS workflow, click **Wind** beside **Scenario**, edit world
XYZ mean velocity (m/s), gust standard deviations (m/s), correlation time (s)
and seed, then **Apply wind**. +Z points up; the vector points toward air travel.
Opening/closing the panel does not change wind. Apply works while stopped without
stepping, or live during flight. Start, Reset and spawn changes retain settings
and restart gust history; restarting physics reloads YAML defaults. The spectator
windsock shows actual mean-plus-gust velocity and is excluded from drone images.
See [runtime wind behavior](docs/design/aerodynamics.md#runtime-wind-updates).

### Recorded flight diagnostics

The standard workspace **Bullet: Run all** task now records each external-RC
flight automatically in MCAP, with separate process logs per launch. Actual ground
truth, available sensor outputs, effective configuration and wind are recorded
independently of render-pose selection. See [telemetry design](docs/design/telemetry.md#external-rc-flight-recording-adapter)
and the sibling recorder README for the offline clipping report command.
