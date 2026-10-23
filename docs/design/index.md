# Design menu

- Library execution: applications and examples own reset/step/close loops, pacing,
  observation consumers, and cleanup; the package has no shared CLI or runner.
- [Configurable aerodynamics](aerodynamics.md): independent force models, wind,
  local wakes, collision-surface queries and ideal-mode compatibility.
- [Shared frame math](frame-math.md): quaternion conventions, validation,
  source-state diagnostics and controller error translation.
- [IMU and barometer sensors](sensors.md): measurement frames, noise configuration,
  reset/step lifecycle and observation/telemetry interfaces.
- [Simulation telemetry](telemetry.md): per-topic schemas, shared headers, YAML
  selection, optional example-side streaming and the future recorder strategy.
- [PlotJuggler live position](../../plotjuggler/README.md): UDP setup, two-panel
  layout, isolated project profile and VS Code tasks;
  [sender ownership/config](../../tools/telemetry_sender/README.md).
- [Drone identity and telemetry](drone-identity-and-telemetry.md): configured
  drone IDs, episode identity, shared header identity and compatibility.
- [Position and velocity control](position-control.md): separate XY PID interfaces,
  Z composition, RC mapping, frames, lifecycle and square-flight verification.
- [Implemented control chain](../forces/drone_control.svg): visual command/feedback flow.
- [Force and effect implementation map](../forces/force-implementation-map.md):
  source review, current status, future ownership and independent checks.
- [Numbered force-chain tips](../forces/force-implementation-chain.svg):
  hover/click badges mapped to the force table indices.

This checkout does not include the older architecture or force-roadmap
documents referenced in repository guidance. This menu documents the current
library interfaces rather than reconstructing those missing records.

- [Godot / ZeroMQ camera](zmq-rendering.md): Godot Start/Stop/Reset, physics-paced poses, TCP signaling and local SHM images.
- [App flight channel](app-flight-channel.md): external RC lease, readiness,
  sensor-only feedback, expiry watchdog and exact capture-source timestamps.
