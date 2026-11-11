# Design menu

- Library execution: applications and examples own reset/step/close loops, pacing,
  observation consumers, and cleanup; the package has no shared CLI or runner.
- [Configurable aerodynamics](aerodynamics.md): independent force models, wind,
  local wakes, collision-surface queries and ideal-mode compatibility.
- [Shared frame math](frame-math.md): quaternion conventions, validation,
  source-state diagnostics and controller error translation.
- [IMU, barometer and GPS sensors](sensors.md): measurement frames, noise configuration,
  reset/step lifecycle and observation interfaces.
- [Simulation ↔ Betaflight SITL bridge](betaflight-sitl-bridge.md): upstream-target
  FDM sensor fields, UDP ports, motor order, timing and implementation status,
  with packet tables and SVG drawings.
- [Standalone Betaflight altitude hold](../../examples/altitde_hold_bt/README.md):
  independent mission, MSP RC, owned firmware lifecycle and PyBullet execution.
- [Betaflight SITL terrain following](../../examples/terrain_following_bt/README.md):
  rangefinder-based 3 m terrain clearance, Acro rate control, and owned firmware lifecycle.
- [Drone identity](drone-identity.md): configured drone IDs and per-reset episode
  identity.
- [Vehicle URDF selection](vehicle-urdf.md): packaged model aliases, custom
  four-rotor URDF paths, required asset contract and YAML path resolution.
- [Position and velocity control](position-control.md): separate XY PID interfaces,
  Z composition, RC mapping, frames, lifecycle and square-flight verification.
- [Implemented control chain](../forces/drone_control.svg): visual command/feedback flow.
- [Packaged external renderer](../../examples/altitude_hold_external_render/README.md):
  automatic altitude hold, Angle square, and independent Betaflight Acro square
  flights; owned exported Godot viewer, image readiness, local communication and cleanup.
- [Force and effect implementation map](../forces/force-implementation-map.md):
  source review, current status, future ownership and independent checks.
- [Numbered force-chain tips](../forces/force-implementation-chain.svg):
  hover/click badges mapped to the force table indices.

This checkout does not include the older architecture or force-roadmap
documents referenced in repository guidance. This menu documents the current
library interfaces rather than reconstructing those missing records.

- [External renderer interface](external-render-interface.md): process ownership, configuration, ZeroMQ topics, pose/image data, readiness, and cleanup.
- [Godot / ZeroMQ camera](zmq-rendering.md): Godot Start/Stop/Reset, physics-paced poses, TCP signaling and local SHM images.
- [App flight channel](app-flight-channel.md): external RC lease, readiness,
  sensor-only feedback, expiry watchdog and exact capture-source timestamps.

- [MAVLink terrain monitor](../../examples/terrain_following_mavlink/README.md): separate MAVLink telemetry connection with shared MSP Acro mission. The terrain runner accepts an optional SITL session factory; the base session accepts a profile path and additional reserved TCP ports.

- [Local protocol libraries](../../libs/README.md): simulation contracts and independent image transport, packaged as separate wheels.

- [Rendered takeoff and SHM image viewer](../../examples/takeoff_image_viewer/README.md): separate camera process with bounded frame polling and owned lifecycle.

- [Live wind UI example](../../examples/wind_control/README.md): example-owned wind editor, step-boundary application and renderer wind-state publication.

- [GPS point-to-point MAVLink feedback](../../examples/gps_point_to_point_mavlink/README.md): standalone 400 m GPS line flight, terrain-map landing, firmware telemetry feedback and MSP Acro commands.
