# Position square with automatic external rendering

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

## Goal
Fix the square CLI waiting on the ground for a Godot Start command, and bind it to the packaged external renderer.

## Implementation
1. Reproduce the CLI dependency on manual control with a regression check.
2. Move the existing owned renderer adapter to external_render and update callers.
3. Launch the viewer automatically, wait for the initial image, then run the paced square mission. Headless runs without rendering.
4. Update affected tests, task description, and documentation.

## Interfaces and scope
Add --renderer with the packaged executable default; use the packaged render configuration. Preserve optional telemetry and programmatic control support. No controller tuning, firmware, dependency, or Acro behavior changes. No commit or push requested.

## Verification
- Added a headless CLI regression which failed on the old mandatory renderer import and now completes without renderer/control dependencies.
- Affected suite: 103 passed, one pre-existing PlotJuggler layout test failed because `plotjuggler/layouts/ground_truth_position.xml` is absent. No unrelated layout/task repair included.
- Real exported Godot viewer flight with NVIDIA environment: automatic takeoff, all five waypoints settled, hold, landing and disarm completed in 54.371 simulated seconds at 1.000x. Final position approximately (0.000095, 0.001759, 0.013490) m, zero publisher queue drops.
- Owned viewer exited; TCP ports 5555/5556/5557 were reusable afterward. Existing renderer failure/SHM cleanup tests passed.
- Both position-square README Mermaid blocks parsed and rendered successfully; the sequence participant uses DemoLoop instead of the reserved Loop ID.
- `git diff --check` passed. Full repository suite and other platforms were not checked.
