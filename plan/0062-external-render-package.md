# Packaged Godot viewer and standalone altitude hold

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

## Goal

Export the existing renderer for Ubuntu/Linux x86_64 into external_render and
add an independent altitude_hold_external_render example. Its internal controller
climbs to 3 m, holds for 10 s, lands and exits automatically. No runtime Godot
editor, application, recorder or Betaflight process is required.

## Implementation

1. Add a reproducible export recipe, export preset, config and ignored generated
   distribution under external_render/dist/linux-x86_64. Bundle scene/terrain,
   native extension and non-system ZeroMQ dependencies with a relative launcher.
2. Add viewer-only behavior to the sibling Godot scene without changing its
   interactive default. Hide controls, disable collision commands, accept an
   external transport profile, and allow the normal system OpenGL adapter.
3. Add a separate CLI, mission and paced physics loop. Reuse library controllers
   and publisher, with no imports from other examples. Launch the package, publish
   reset pose, and wait up to 10 s for a valid camera image before starting flight.
4. Own renderer/physics/transport cleanup, fail on renderer exit or publication
   failure, and report missing executable, unavailable ports and startup timeout.
5. Add the VS Code choice and READMEs including a communication diagram, startup
   instructions, package contents, platform assumptions and cleanup behavior.

## Interfaces and defaults

New example: examples.altitude_hold_external_render.main. Renderer is viewer-only
and simulation uses ground-truth feedback with the existing altitude cascade.
Keep ZeroMQ 5555 poses/config, 5556 images/SHM and 5557 adapter control endpoint;
the example consumes no control commands. Profiles may be overridden. Packaging
is build-time source dependent, but the exported runtime uses only its own files,
Ubuntu system runtimes and graphics drivers. No new protocols or dependencies.
Generated binaries stay out of Git. No Windows, remote rendering, hardware,
commit or push. Preserve concurrent Betaflight-example work from the main thread.

## Verification

- `uv run --locked python external_render/build.py` exported Godot 4.7.2 release
  with embedded project/terrain/config data, the native adapter and non-system
  ZeroMQ dependencies. Distribution is approximately 85 MiB; Git ignores it.
- `uv run --locked --extra render python -m examples.altitude_hold_external_render.main`
  automatically flew the complete mission and exited. A copy of the final package
  relocated under `/tmp` also passed with only `dirname` on runtime PATH (no Godot
  editor), using the package's own camera/transport profile and a temporary cwd.
- Final relocated run: 10.0 s hold, 0.02615 m peak hover error, final altitude
  0.01349 m, all motors off, completion at 32.825 simulated seconds. First stamped
  RGB8 image was validated and visually inspected; Intel/Mesa OpenGL rendered
  the scene. No renderer/terrain source-tree paths were needed by the executable.
- Found that signal-based exported-engine shutdown bypasses native destructors.
  Added regression coverage first, then cleanup of only the owned child's SHM
  sessions. Startup ownership logs are flushed in the exported project. Final
  live run verified process/thread shutdown and removal of all owned SHM slots;
  a separate live renderer interruption check also passed.
- **14 new tests passed**: mission/hover/landing, missing executable, all three
  occupied ports, startup timeout, early renderer exit, interruption, simulation
  cleanup on failure, owned-versus-foreign SHM cleanup, bundled-library sonames and
  CLI help/errors without physics startup.
- **44 existing example/physics/control/render tests passed**, including the
  standalone altitude-hold mission and square-example Start/Stop/Reset behavior.
  **25 sibling native-adapter tests passed**. Real `test_scene.gd` also passed
  normals, RGB/channel orientation, immediate pose/readback, FOV, UI exclusion
  and truck-location checks with graphics enabled.
- Both new README Mermaid blocks parsed/rendered with the installed Mermaid
  bundle and were visually inspected. Documentation links, task JSON, module
  compilation and `git diff --check` passed in both affected repositories.
- Final diagnostic report: `/tmp/external-render-relocated-9vqfmzqc/report.json`;
  camera PNG and renderer startup log are alongside it. Build/test logs remain
  under `/tmp/external-render-*.log`. No full unrelated workspace suite, Windows,
  remote rendering, hardware, commit or push was performed.
