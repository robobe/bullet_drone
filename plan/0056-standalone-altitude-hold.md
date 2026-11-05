# Standalone altitude hold without Godot

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

Goal: Make altitude hold start immediately in PyBullet, complete its mission and
exit without Godot, ZeroMQ or the render extra. Leave other examples unchanged.

Steps:
1. Add a regression running the headless CLI with Godot/ZeroMQ imports blocked.
2. Remove render configuration from this CLI and renderer/control entry wiring.
3. Remove external-control branches and the control argument from its demo loop;
   retain mission behavior, callbacks, pacing, timeout/error handling and cleanup.
4. Remove altitude hold from Godot-specific test parametrizations and update
   documentation and the task description for its standalone startup.
5. Run the CLI without the render extra and verify takeoff, hold, landing and
   cleanup with the affected tests, including square-example control regressions.

Interfaces: `--render-config` and `run_demo(control=...)` are removed from altitude
hold. GUI runs remain paced; headless defaults to unpaced. Shared rendering
dependencies, square/external-RC examples, flight tuning and physics stay intact.

Verification:
- The blocked-import headless CLI regression failed before the change on
  `tools.zmq_render` and passed afterward. It blocks ZeroMQ, bullet_msgs and
  tools.zmq_render throughout an actual complete mission.
- `uv run --locked pytest tests/test_altitude_hold_example.py tests/test_example_timing.py tests/test_simulation_control.py tests/test_zmq_render.py -q`:
  **44 passed**. Covers 3 m hover accuracy, 10 s hold, landing, disarming,
  completion deadlines, reset/step/PID failure and interruption cleanup, GUI
  pacing, removed-option rejection and the square examples' Godot lifecycle.
- `uv run --locked python -m examples.altitude_hold.main --headless` completed
  without the render extra: takeoff at 0.500 s, hold at 5.487 s, land at 15.487 s,
  disarm at 31.825 s and completion at 32.825 s with final altitude 0.0135 m.
- The example-picker JSON and altitude-hold selection are valid;
  `git diff --check` passed. No real GUI/interactive picker or hardware flight
  was tested. No commit or push is included.
