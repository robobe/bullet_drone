# Betaflight SITL terrain following

Status: Implemented and verified; committed. Implementation commit: `abfc209b89962e49f4097139abbac96ee7247fb2`.

## Goal

Add a standalone Betaflight SITL example that yaws 90 degrees counterclockwise during takeoff, then follows the rising hill along world +Y at a 3 m rangefinder clearance to its crest near `(0, 65)` and retraces the route home before landing and disarming.

## Implementation steps

1. Add independent CLI, mission, SITL lifecycle, diagnostics, runner, and README under `examples/terrain_following_bt`.
2. Load `custom/terrain/terrain_heightmap.f32` and its adjacent `terrain.obj`, enable the simulated rangefinder, and use measured clearance and its sample-to-sample rate in the outer vertical control loop.
3. Wire optional renderer lifecycle and add the example to the VS Code example picker.
4. Add focused tests for terrain mission/control behavior, invalid rangefinder readings, cleanup, and CLI/import behavior.

## Interface changes

None to the library. Adds `python -m examples.terrain_following_bt.main` with `--headless`, `--sitl-binary`, `--renderer`, and `--render-config` options.

## Verification

- `uv run --locked --extra sitl pytest -q tests/test_terrain_following_bt_example.py` — 7 passed.
- Paced headless SITL flight completed the +Y climb to the Y=65 m crest, retraced home, landed, and disarmed in 147.042 simulated seconds.
- Paced NVIDIA-rendered SITL flight completed the same route, landed, disarmed, and closed the viewer with zero publisher queue drops in 146.079 simulated seconds.
- PyBullet reset confirmed terrain collision loads and the initial rangefinder reading is valid at 1.5 cm. The measured clearance briefly dipped to 2.36 m at the steepest face and recovered near the 3 m target.
- CLI help, task JSON, Mermaid rendering, and `git diff --check` passed.

## Scope and defaults

Use the tracked custom terrain assets, racer vehicle, 240 Hz physics, current Betaflight SITL profile, +90 degree yaw target, 3 m clearance, and route `(0,0) → (0,15) → (0,30) → (0,45) → (0,60) → (0,65) → (0,60) → (0,45) → (0,30) → (0,15) → (0,0)` in world metres. No separate obstacles or obstacle-avoidance behavior. Example mission/control/lifecycle code must not import another example's implementation.

## Implementation commit

`abfc209b89962e49f4097139abbac96ee7247fb2`
