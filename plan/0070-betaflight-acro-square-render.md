# Betaflight Acro square with external rendering

Status: In progress. Implementation commit: Pending.

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

Pending. No commit or push requested.
