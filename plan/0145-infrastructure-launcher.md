# Add a simulation, SITL, and renderer launcher

## Goal
Provide one main entry point for a persistent PyBullet FDM loop, optional Betaflight SITL, and optional Godot rendering. The launcher never opens MSP; a separate controller owns MSP and optional MAVLink connections. Ctrl-C stops the loop and closes owned processes.

## Changes
- Add a Click CLI with `--sitl/--no-sitl` and `--render/--no-render`, both defaulting on, plus binary, renderer and render-config paths.
- Add a process-only SITL session mode reusing existing firmware validation, config boot, port ownership, logging, exit detection, and teardown without creating an MSP client.
- Run a 240 Hz paced PyBullet loop. With SITL enabled, send FDM and consume motor packets through `BetaflightBackend`; with SITL disabled, hold the vehicle stationary. Publish each observation when rendering is enabled.
- Start SITL, then renderer, then physics. Clean up in reverse order on Ctrl-C and errors.
- Document the launcher command and that an external process owns MSP/MAVLink.

## Verification
Cover CLI defaults and flags, process-only SITL ownership, loop behavior without SITL, and cleanup on Ctrl-C. Run a rendered SITL launch and confirm no MSP socket is opened by launcher.

## Status
Implemented; unit checks passed, process-only SITL Ctrl-C verified without an MSP client. Rendered startup remains for manual verification.

## Implementation commit
Pending.
