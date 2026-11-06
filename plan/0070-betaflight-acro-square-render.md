# Betaflight Acro square with external rendering

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

## Goal

Replace the internally stabilized, command-driven `position_square_acro` example
with a self-contained Betaflight SITL Acro square that automatically launches the
packaged external renderer and takes off.

## Implementation steps

1. Add a regression that rejects imports from other examples and requires local
   SITL, renderer, mission, and diagnostics modules.
2. Implement an example-owned square mission that converts world position and
   velocity targets into Acro body-rate and throttle MSP channels.
3. Own the unmodified Betaflight 2026.6.2 process, MSP RC/status connection,
   FDM/motor bridge, PyBullet environment, pacing, diagnostics, and cleanup.
4. Own an example-local exported-renderer adapter, wait for its first image, then
   start the mission automatically. Keep `--headless` renderer-free.
5. Replace obsolete internal-controller/control-channel tests and update the CLI,
   design documents, diagrams, and VS Code task description.
6. Verify unit/lifecycle behavior, render Mermaid blocks, and run a real paced
   SITL flight with the packaged renderer and NVIDIA environment.

## Interfaces and scope

The Acro CLI gains `--sitl-binary` and `--renderer`; `--render-config` uses the
packaged external-render profile. The example imports project library modules but
no other example modules. Betaflight closes the body-rate and motor loops; the
example retains external XYZ navigation. No firmware patch, shared-example helper,
dependency, or Angle-square behavior is included. The profile explicitly records
the existing default roll/pitch ACTUAL-rate values used by the inverse mapping;
this does not retune those defaults.

## Verification

- New Acro square tests: 11 passed; focused cross-example suite: 74 passed,
  3 opt-in tests skipped. The firmware profile regression checks the explicit
  roll/pitch/yaw ACTUAL-rate curve values.
- Full available suite excluding the missing optional `plotjuggler` package:
  729 passed, 3 skipped, and 2 unrelated existing failures. PlotJuggler layout
  test references missing `plotjuggler/layouts/ground_truth_position.xml`;
  sensor independence test expects no rangefinder although `configs/basic.yaml`
  enables it.
- Real headless firmware flight completed: armed, settled all five waypoints,
  held 10 seconds, landed and disarmed at 59.062 simulation seconds. Final
  position `(0.00045, -0.00020, 0.01349)` m; SITL process closed.
- Real NVIDIA viewer flight completed at 58.979 simulation seconds. Final
  position `(-0.00053, 0.00001, 0.01349)` m; 1,770 frames scheduled, zero queue
  drops. Viewer and SITL processes exited, and ports were released.
- Both README Mermaid diagrams parsed and rendered. `git diff --check` and Python
  compilation passed.
- Full-suite collection with all workspace extras was blocked by unavailable
  optional `plotjuggler`; no commit or push requested.
