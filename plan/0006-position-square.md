# Position/velocity split and 3 m square

## Goal
Separate horizontal position and velocity PIDs and create an own-loop RC angle-mode
example that flies a settled 3 m square at altitude 3 m, holds home and lands.

## Implementation steps and interfaces
- Replace combined PositionController.input with velocity(target XY, measured XY,
  dt), returning bounded world XY velocity. Remove PositionControlOutput.
- Add VelocityController.update(target velocity XY, measured velocity XY,
  vertical thrust N, measured yaw rad, dt), returning immutable acceleration XY,
  roll/pitch radians and bounded collective N. Keep acceleration-to-angle conversion,
  tilt limiting and tilt compensation here. Z remains in AltitudeController.
- Example composes all stages and yaw heading_rate, encodes RcInput, and owns
  reset/step, GUI pacing and cleanup. Separate parsing, mission, composition, GUI.
- Migrate tests/exports/docs, add example README diagrams, VS Code tasks, focused
  design documentation, and update the control SVG.

## Scope/defaults
- Start (0,0,0.015), disarmed settle 0.5 s, take off to world Z=3.
- Route (3,0,3), (3,3,3), (0,3,3), (0,0,3); hold initial heading.
- Arrival: XYZ error ≤0.1 m and total speed ≤0.1 m/s continuously for 1 s.
- Hold home 10 s; descend toward Z=0 at commanded speed ≤0.5 m/s with XY active.
- Touchdown: XY error ≤0.1 m, Z≤0.02 m, |vertical speed|≤0.05 m/s for 0.5 s.
- Disarm for 1 s then close; timeout 90 s. GUI default, --headless available.
- Reuse altitude/stabilization tuning values, 240 Hz, 1 kg and gravity 9.81.
- XY position PID (.8,0,0), speed limit 1 m/s; velocity PID (2,0,0),
  acceleration limit 1.5 m/s²; tilt limit 15°. Named constants after imports.
- Intentional position interface break. No dependencies, joystick hardware,
  sensors, Betaflight or altitude-example behavior changes; no commits/pushes.
- Referenced design menu is currently absent; create focused current documentation.

## Verification
Test controllers through their interfaces: direction, vector limits, yaw-frame
conversion, tilt/thrust limits, reset, immutable outputs, invalid input. Test mission
dwell interruptions, full headless chain/waypoints/heading/landing/motors off,
deadline boundaries, failure/interruption cleanup and import/help safety. Run full
suite and inspect paced GUI and updated SVG.

## Status
Implemented and verified; uncommitted.

Verification outcome:
- Full suite: `uv run --locked pytest -q` → 285 passed, 1 skipped (optional telemetry).
- Controller and new mission checks: 42 passed before the final full-suite run.
- Normal headless and paced GUI flights completed at 44.496 simulated seconds.
  Waypoints settled at 5.179, 10.496, 15.812, 21.129 and 26.446 s; landing began
  at 36.446 s and disarmed at 43.496 s. Final position approximately
  (0.00284, -0.00432, 0.01349) m with stopped rotors.
- Full-flight integration test verifies every armed step from controller outputs
  and RC quantization through attitude/rate feedback, mixer, motor speeds and
  applied forces/torque. Waypoint, hover, heading, landing and disarm criteria pass.
- Exact-deadline completion and one-timestep-earlier timeout pass; injected reset,
  step, position, velocity and altitude failures/interruptions release resources.
- Import/help safety passed. Square GUI captured during the second leg and
  visually inspected; camera covers flight range. Complete normal GUI run exited 0.
- Updated SVG rendered and visually inspected; SVG XML and VS Code task JSON
  parse successfully. Static scan found no unused imports or executable callers
  of the removed combined controller interface.
- Dependencies unchanged; optional telemetry integration not exercised.

Implementation commit hash: Pending (no commit requested).
