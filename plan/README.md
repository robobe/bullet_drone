# Implementation plans

| Plan | Description | Status | Implementation commit |
| --- | --- | --- | --- |
| [0003: control diagram SVG](0003-control-diagram-svg.md) | Replace obsolete PNG with current RC/internal-control diagram. | Implemented and verified; uncommitted | Pending |
| [0004: named constants guidance](0004-named-constants-guidance.md) | Require descriptive constants grouped after module imports. | Implemented and verified; uncommitted | Pending |
| [0005: altitude hold constants](0005-altitude-hold-constants.md) | Replace example magic numbers with named module constants. | Implemented and verified; uncommitted | Pending |
| [0006: position square](0006-position-square.md) | Separate XY position/velocity PIDs and fly a settled 3 m square through RC angle mode. | Implemented and verified; uncommitted | Pending |
| [0007: retire shared CLI/runner](0007-retire-cli-runner.md) | Remove shared execution entry point and dedicated tests; retain library verification and example loops. | Implemented and verified; uncommitted | Pending |
| [0008: Acro position square](0008-position-square-acro.md) | Fly the same square using upstream angle-to-rate conversion and Acro RC input. | Implemented and verified; uncommitted | Pending |
| [0009: force implementation chain](0009-force-chain-mapping.md) | Map course effects to implementation stages with a numbered table and SVG tips. | Complete; committed | `ac453654b7378854b8e449280df88016a2ab556b` |
| [0010: drone ID telemetry](0010-drone-id-telemetry.md) | Propagate configured drone identity into observations and MCAP schemas/payloads. | Complete; committed | `65830c3a85a7928ddcabf28cb6615be2e28fa187` |
| [0011: IMU and barometer](0011-imu-barometer-sensors.md) | Configurable sensor classes, typed observations and MCAP payloads. | Complete; committed | `84449a54c030f9393b41b8048bf31e9cd8caa381` |
| [0012: YAML configuration comments](0012-yaml-configuration-comments.md) | Require explanatory comments for every YAML configuration field. | Complete; committed | `5d9375636df529290aaecc92f450062fcc18b044` |
| [0013: basic YAML comments](0013-basic-yaml-comments.md) | Explain every existing basic configuration field with adjacent comments. | Complete; committed | `5d9375636df529290aaecc92f450062fcc18b044` |
| [0014: shared frame math](0014-shared-frame-math.md) | Give quaternion/frame math neutral ownership and preserve controller error behavior. | Complete; committed | `3f92fe2fbcde954e2ece0eee279424380325ae86` |
| [0015: real-time factor](0015-realtime-factor-logging.md) | Log simulated seconds per wall-clock second in all three examples. | Complete; committed | `eb83baa1dd7bc05b6d49a34b528815adcdb1f9c7` |
| [0016: configurable aerodynamics](0016-configurable-aerodynamics.md) | Add independent aerodynamic models, default-enabled YAML controls and repeatable wind. | Complete; committed | `7619e7dea916908c387b36ef22f404dafa600406` |
| [0017: telemetry recorder TODO](0017-telemetry-recorder-todo.md) | Document the future centralized simulation telemetry recorder milestone. | Complete; committed | `0e629400438b6f1d2fcbb4e37b40853ed3108aa7` |
| [0018: simulation telemetry schemas](0018-simulation-telemetry-schemas.md) | Replace MCAP with per-topic schemas, a common header and YAML topic selection. | Complete; committed | `0e629400438b6f1d2fcbb4e37b40853ed3108aa7` |
| [0019: PlotJuggler sender](0019-plotjuggler-sender.md) | Stream ground truth through a sender process with YAML, UDP layout and VS Code tasks. | Complete; committed | `dd15909896b0bb9a3a0590a6025700a9ec938f7d` |
| [0020: PlotJuggler profile](0020-plotjuggler-profile.md) | Save UDP timestamp preferences and use an isolated profile from the viewer task. | Complete; committed | `dd15909896b0bb9a3a0590a6025700a9ec938f7d` |
| [0021: Godot Zenoh rendering](0021-godot-zenoh-render.md) | Optional physics-driven camera poses, Godot bridge and SHM recording. | Complete; committed | `715c9fe249ad01795df73e72c0cd3544cfb07a7c` |
| [0022: Godot ZeroMQ rendering](0022-godot-zmq-render.md) | Local IPC transport experiment and NVIDIA performance comparison. | Complete; committed | `f9f4cc95c1b38a06b3d5d0c90d1f8b2755abfa1a` |
| [0023: ZMQ TCP and SHM](0023-zmq-tcp-shm.md) | Shared contracts, local TCP signaling, SHM images and live viewer. | Complete; committed | `2ec58b5318df28a2de180e0574358b4ae39cf2fd` |
| [0024: Godot simulation controls](0024-godot-simulation-controls.md) | PUB/SUB Start/Stop/Reset, stopped startup and authoritative UI state. | Complete; committed | `2ec58b5318df28a2de180e0574358b4ae39cf2fd` |
| [0025: PlotJuggler altitude window](0025-plotjuggler-altitude-window.md) | Save 0–5 m altitude bounds and a 360-second opening window/buffer; document live auto-fit. | Complete; committed | `20652a1a2f3bfbdc321f3d29340e8ed1612e7bd9` |
| [0026: pose ground-truth velocity](0026-pose-ground-truth-velocity.md) | Publish world linear velocity in pose v2 with shared/native validation and MCAP readback. | Complete; committed | `51e37e60078dc27a0996f3830acf80721b739a2b` |
| [0027: shared workspace environment](0027-shared-workspace-environment.md) | One Python 3.12 uv environment, lockfile and build/run configuration across six sibling projects. | Implemented; verified with Godot 4.7 | `b147bb5b4949bba649d178e8c144f15bf845421e` |
| [0028: app flight channel](0028-app-flight-channel.md) | App-controlled 20 m takeoff, settled hover and LOS approach with expiring RC channel. | Complete; committed | `fac9ee367cf79f7a4a11fbd88f12c925467022cf` |
| [0029: renderer collision completion](0029-render-collision-end.md) | Truck contact, authoritative scenario completion, app latch and terminal MCAP drain. | Complete; committed | `aecc8312cea2cea622fd08b893b51d9be794ffbb` |
| [0030: LOS telemetry in degrees](0030-los-telemetry-degrees.md) | Focused 30Hz bearing/rate/TTC/confidence/state stream from app to MCAP. | Complete; committed | `610c92bb54fcfc39df37af0b532515e2b2354da6` |
| [0031: spawn scenarios](0031-spawn-scenarios.md) | Landscape picking, world altitude, saved presets and confirmed stopped spawn configuration. | Complete; committed | `1163fc93b80b7753cfc7528eeb4ff77718f75dab` |
| [0032: scenario takeoff height](0032-scenario-takeoff-height.md) | Configurable relative climb with preset persistence and automatic terrain-based starting Z. | Complete; committed | `1163fc93b80b7753cfc7528eeb4ff77718f75dab` |
| [0033: LOS envelope errors](0033-los-envelope-errors.md) | Allow +/-35 degree azimuth and expose rejected bearing, limits and physics stop reasons in mission logs. | Complete; committed | `f6e5e302242388f337fee28aff70690f316d43c6` |
| [0034: yaw target search](0034-yaw-target-search.md) | One measured yaw circle, candidate braking/settling and continuous LOS handoff at the new heading. | Complete; committed | `0861c8cfd6d0442773998500200092bd2660d87b` |
| [0035: center target before LOS](0035-center-target-before-los.md) | Proportional yaw centers the bbox horizontally before settling and ending search. | Complete; committed | `fca7f74d8d8fbf6d74c537420d5e4ea8a44c49e5` |
| [0036: single workspace task](0036-single-workspace-task.md) | Replace workspace and folder tasks with one four-service launch task. | Implemented; stub launch/cleanup verified | Pending |
| [0037: sensor pose sources](0037-sensor-pose-sources.md) | YAML-selected barometer altitude and complementary-filter IMU attitude for camera/app. | Implemented; 2 unrelated legacy task-test failures | `7c6e86617a25fc300adfaa1a3116c225750f78fc` |
| [0038: startup configuration banner](0038-startup-config-banner.md) | Print all resolved external-RC startup settings and source paths to stdout. | Implemented; 17 focused checks passed | `55562afcc05b73de4b76ab2cbe165de15b276806` |
| [0039: barometer noise profile](0039-barometer-noise-profile.md) | Enable representative BMP388 pressure noise in the basic YAML profile. | Implemented; 95 focused checks passed | `ce54722cebbcea1ccac78eb70008de4b5e23c14f` |
| [0040: barometer pressure filter](0040-barometer-pressure-filter.md) | Optional sensor pressure smoothing before altitude conversion. | Implemented; 290 focused checks passed | `ce54722cebbcea1ccac78eb70008de4b5e23c14f` |
| [0041: noise-tolerant hover transition](0041-noise-tolerant-hover-transition.md) | Use sensor altitude trend to confirm hover and yaw settling without control changes. | Implemented; 83 app checks passed | `9b85374f70a0e87ad7f8d873922847e540258d66` |
| [0042: ICM-42688-P IMU noise](0042-icm42688p-imu-noise.md) | Density-based IMU noise, sensor filters and configurable residual bias drift. | Implemented; 420 relevant checks passed | `5842d499d976f2d0945c185e5d9a485b63e2accd` |
| [0043: physical rotor vibration](0043-physical-rotor-vibration.md) | Resolved rotor forces and offline tracking impact experiment. | not-imp (not implemented) | Pending |
| [0044: live wind controls](0044-live-wind-controls.md) | Physics-owned live wind settings, Godot panel and spectator windsock. | Implemented; runtime and visual checks passed | `f292b468846df7e0d3ac076ac98c0f7bc460b907` |
| [0045: flight analysis recording](0045-flight-analysis-recording.md) | Automatic flight MCAP, exact frame matching and offline failure diagnostics. | Implemented; recording and analysis checks passed | `8cde477426dc12f2c6be12542487a9edd50f8890` |

- [0046: Betaflight MSP backend](0046-betaflight-msp-backend.md) — Complete, implementation commit `19d3247`.
- [0047: Betaflight motor mapping check](0047-betaflight-motor-mapping-check.md) — Pending, implementation commit Pending.
