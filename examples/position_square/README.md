# Position-controlled 3 m square

This example owns its simulation loop and flies in RC angle mode. It takes off
to world Z = 3 m, flies a 3 m square, settles at every corner, returns home, holds
for 10 simulated seconds, then lands and disarms.

Run from the repository root:

```bash
uv run --locked --extra render python -m examples.position_square.main
uv run --locked --extra render python -m examples.position_square.main --headless
uv run --locked --extra render python -m examples.position_square.main --help
```

The default command opens the packaged Godot viewer with PyBullet headless,
waits for its first image, and starts the mission automatically. It needs a
working desktop/OpenGL display and an exported renderer. Build it once:

```bash
uv run --locked python external_render/build.py
```

Use `--renderer /path/to/render` to select another exported executable and
`--render-config /path/to/physics.json` for another camera/transport profile.
Defaults live under `external_render/dist/linux-x86_64/render` and
`external_render/configs/physics.json`. The shared adapter in
`external_render/renderer.py` owns Godot, ZeroMQ, and cleanup.

`--headless` runs the same complete flight without windows, rendering dependencies,
or pacing. Ctrl+C closes resources and returns 130. Timeout/runtime failure
returns 1; invalid configuration/arguments returns 2. Only one rendered example
can own the shared ports at a time. Closing the viewer stops the flight and
releases physics and transport resources.

## Flight sequence

The drone starts at `(0,0,0.015)` m. It settles disarmed for 0.5 s, then visits
`(0,0,3) → (3,0,3) → (3,3,3) → (0,3,3) → (0,0,3)` in world meters.
The mission holds the heading captured at reset. Each arrival requires XYZ error
≤0.1 m and total speed ≤0.1 m/s continuously for 1 s. A failed condition resets
the dwell timer. The final home hold lasts 10 simulated seconds after arrival.

Landing targets home XY and Z = 0, retaining horizontal position control and
limiting the vertical speed target to 0.5 m/s. Touchdown requires XY home error
≤0.1 m, Z ≤0.02 m and vertical speed ≤0.05 m/s continuously for 0.5 s. Motors
then disarm; physics continues for 1 s before closing. The overall simulation-time
limit is 90 s, including the final permitted observation before deciding timeout.

## Control stages and tuning

`demo.py` wires the modules and owns the loop, `mission.py` owns flight phases,
`demo_cli.py` only parses, and `gui.py` frames the viewer. Constants are grouped
after imports in each owning module. The Betaflight Acro square has an independent
mission and execution loop.

Each iteration uses the latest reset/step `DroneObservation.ground_truth`:

1. `PositionController.velocity()` maps XY position error to world XY velocity.
2. `AltitudeController.thrust()` maps Z/vertical-speed feedback to vertical thrust.
3. `VelocityController.update()` maps XY velocity error to acceleration, then to
   yaw-aware roll/pitch angles and tilt-compensated collective thrust.
4. `heading_rate()` maps heading error to a bounded yaw rate.
5. `RcCodec.encode()` maps angles/yaw rate/thrust and arm state to 18 RC channels.
6. `RcSimulation.step()` stabilizes attitude/rates, mixes motors, applies rotor
   forces, and returns the next physics observation.

The simulation is 240 Hz, mass 1 kg, and gravity 9.81 m/s². Signals use ideal
ground truth rather than noisy sensors. RC encoding rounds to integer channels.
Thrust capacity comes from the bundled racer, with no scaling above motor limits.

| PID | Kp | Ki | Kd | Output limit | Integral contribution limit |
| --- | --- | --- | --- | --- | --- |
| XY position → velocity | 0.8 | 0 | 0 | 1 m/s, vector magnitude | 0 m/s |
| XY velocity → acceleration | 2 | 0 | 0 | 1.5 m/s², per axis | 0 m/s² |
| Altitude → vertical speed | 1.5 | 0 | 0 | 1 m/s | 0 m/s |
| Vertical speed → acceleration | 6 | 4 | 0 | 3 m/s² | 2 m/s² |
| Body rate → torque | 0.05 | 0 | 0 | 0.1 N·m, per axis | 0 N·m |

Kp units are output/input, Ki output/(input·s), Kd output·s/input. Maximum tilt
is 15°. Attitude gains are 4 (rad/s)/rad and rate limits are 2 rad/s. Collective
includes tilt compensation; if it saturates, vertical force can decrease.
The commanded descent-speed limit does not bound every transient measured speed.

## Sequence diagram

```mermaid
sequenceDiagram
    participant DemoLoop as Example loop
    participant Mission as Square mission
    participant Position as XY position PID
    participant Altitude as Z altitude cascade
    participant Velocity as XY velocity PID + tilt conversion
    participant RC as RC codec
    participant Sim as RC simulation + PyBullet
    DemoLoop->>Sim: reset()
    Sim-->>DemoLoop: Initial ground truth
    DemoLoop->>Mission: reset(observation), capture home and heading
    loop Until complete or timeout
        DemoLoop->>Mission: command(latest observation, dt)
        Mission->>Mission: Update waypoint and simulation-time dwell
        alt Armed flight
            Mission->>Position: Target/measured XY position
            Position-->>Mission: XY velocity target (m/s)
            Mission->>Altitude: Target/measured Z and vertical speed
            Altitude-->>Mission: Vertical thrust (N)
            Mission->>Velocity: XY velocity target/feedback, vertical thrust, yaw
            Velocity-->>Mission: Roll/pitch (rad), collective thrust (N)
            Mission->>Mission: Heading error to yaw rate
            Mission->>RC: Encode angle mode, angles, yaw rate, thrust, armed
        else Disarmed
            Mission->>RC: Encode angle mode, zero thrust, disarmed
        end
        RC-->>Mission: RcInput
        Mission-->>DemoLoop: RcInput
        alt Mission complete
            DemoLoop->>DemoLoop: Finish without another physics step
        else Continue within deadline
            DemoLoop->>Sim: step(RcInput)
            Sim->>Sim: Angle P, rate PID, mixer, motors, propellers, physics
            Sim-->>DemoLoop: Next ground-truth observation
            DemoLoop->>DemoLoop: Publish pose, pace if external viewer or PyBullet GUI
        end
    end
    DemoLoop->>Sim: close() on success, timeout, error, or interruption
```

## State flow diagram

```mermaid
flowchart TD
    A[Ground settle: disarmed] -->|0.5 s| B[Takeoff: target home XY and Z = 3 m]
    B -->|XYZ error <= 0.1 m and speed <= 0.1 m/s for 1 s| C[Square: visit each 3 m corner]
    C -->|Settled for 1 s: next corner| C
    C -->|Settled at home| D[Hold home XYZ and heading]
    D -->|10 simulated seconds| E[Land: hold home XY, target Z = 0]
    E -->|Home XY error <= 0.1 m, Z <= 0.02 m, vertical speed <= 0.05 m/s for 0.5 s| F[Disarmed: motors stopped]
    F -->|1 simulated second| G[Complete and close]
    T[Timeout at 90 s, failure, or Ctrl+C during an active phase] --> X[Close and report failure/interruption]
```

## Verification and interface migration

```bash
uv run --locked pytest -q tests/test_position.py tests/test_position_square_example.py
```

Tests cover separated PIDs, frame conversion, limits, reset and validation; the
full RC/stabilization/mixer/forces chain; dwell interruptions; landing at home;
deadline boundaries; cleanup; safe imports and help. Viewer lifecycle and automatic startup have regression coverage.

The old combined `PositionController.input()` and `PositionControlOutput` are
removed. See [the position-control design](../../docs/design/position-control.md)
for the new interfaces and composition. The altitude-hold example is unchanged.

## Colored progress logs

Phase changes show `STATE previous -> current`: settle/disarmed in yellow,
takeoff in cyan, square in magenta, hold/complete in green, and landing in blue.
Waypoint status shows the zero-based waypoint index, target XYZ in meters, and
simulation time: cyan when active, green when settled. A new active waypoint
is logged when takeoff starts or the target changes. Colors appear in terminal
output; redirected logs remain plain text. Timing messages keep their usual format.

## Real-time factor

Every wall-clock second, and once on successful completion, the example logs
cumulative simulation time, wall-clock time and their ratio:

```text
Timing: sim=10.000s wall=10.002s RTF=1.000x
Final Timing: sim=23.225s wall=23.227s RTF=1.000x
```

`RTF = elapsed simulation seconds / elapsed wall-clock seconds`:

- `1.0x`: one simulated second takes one real second.
- `2.0x`: two simulated seconds pass per real second (twice real-time speed).
- `0.5x`: one simulated second takes two real seconds (half real-time speed).

External-viewer mode paces the loop toward `1.0x`; sustained values below one mean
execution is falling behind. Headless mode has no pacing and may run faster.
Measurements include control, physics, sensors, rendering overhead and pacing
sleep, starting after initialization and ending before cleanup. Values are
cumulative averages, so brief stalls may be smoothed out. If no wall time has
elapsed, the final ratio is `n/a`. This diagnostic measures execution speed,
not physical accuracy, and does not change stepping or pacing.

The examples enable the new teaching aerodynamic models with calm ambient wind.
The altitude velocity PID now uses Kp=6, Ki=4, output limit 3 m/s² and integral
contribution limit 2 m/s² to retain altitude accuracy and land through ground
effect. Mission routes, settling criteria and timeout limits are unchanged.
See [aerodynamic model details](../../docs/design/aerodynamics.md).

## External viewer and control compatibility

The default CLI runs one automatic mission and closes its owned viewer after
landing. No Start button is needed. The Betaflight Acro square also runs one
automatic mission with its own viewer and SITL process. Direct `run_demo()` calls still support an
optional control adapter and PyBullet GUI for existing Python callers.
See [transport ownership](../../docs/design/zmq-rendering.md).

## Control signal chain

The diagrams follow the default Angle-mode example from waypoint targets through
the navigation cascades, RC translation, internal stabilization, and the motor
mixer. They are split into navigation and mixer stages so each signal label is
readable. Navigation runs in the example mission; attitude/rate stabilization
and mixing run inside `RcSimulation`.

### Navigation cascades

![XY and altitude cascades](control-navigation.svg)

### RC, stabilization, and mixer

![RC command to motor mixer](control-mixer.svg)

Every measured value below comes from the latest PyBullet ground-truth
observation; these controllers do not use noisy sensor samples.

| Controller stage | Target / command input | Feedback input | Output |
| --- | --- | --- | --- |
| XY position PID ×2 | Waypoint XY position (m) | Ground-truth XY position (m) | XY velocity target (m/s), limited to 1 m/s vector magnitude |
| XY velocity PID ×2 | Velocity target (m/s) | Ground-truth XY velocity (m/s) | World XY acceleration (m/s²), limited to 1.5 m/s² per axis |
| Altitude position PID | Target altitude (m) | Ground-truth altitude (m) | Vertical speed target (m/s), limited to 1 m/s in flight and 0.5 m/s landing |
| Vertical velocity PID | Vertical speed target (m/s) | Ground-truth vertical speed (m/s) | Vertical acceleration (m/s²) |
| Yaw-aware force conversion | World XY acceleration and vertical thrust | Ground-truth yaw | Roll/pitch targets (rad), collective thrust (N) with tilt compensation |
| Heading hold P | Heading captured at reset (rad) | Ground-truth yaw (rad) | Yaw-rate target (rad/s), gain 4 s⁻¹ |
| Angle P | Roll/pitch targets (rad) | Ground-truth orientation | Body-rate targets (rad/s), limited to 2 rad/s per axis |
| Body-rate PID ×3 | Body-rate targets (rad/s) | Ground-truth body rates (rad/s) | Roll/pitch/yaw torque (N·m), limited to 0.1 N·m per axis |
| Motor mixer | Collective thrust (N) and body torque (N·m) | None | Four motor speed targets (rad/s), bounded by motor limits |

The mission encodes roll/pitch angle, yaw rate, thrust, and arm state as 18 RC
channels. Software RC decoding turns those channels back into an angle-mode
command. Collective thrust goes directly to the mixer; the Angle P and body-rate
PID stages produce the torque corrections that the mixer allocates across the
four motors.
