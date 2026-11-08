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
- [0048: Heavy quad propulsion upgrade](0048-heavy-quad-propulsion-upgrade.md) — Implemented; focused tests passed, implementation commit `4de86bd`.
- [0049: Periodic render profile](0049-periodic-render-profile.md) — Implemented; focused publisher and adapter checks passed, implementation commit `53b96b1`.
- [0050: Shared terrain heightmap and rangefinder](0050-terrain-heightmap-rangefinder.md) — Implemented and verified, implementation commits `6fb1bd5`, `a796b0d`.
- [0051: Preflight terrain clearance](0051-spawn-terrain-clearance.md) — Implemented; focused tests passed, implementation commit `0f60657`.

- [0052: Unpatched Betaflight SITL integration](0052-unpatched-sitl.md) — Implemented; live upstream and canonical hover verified with test limitations recorded in 0054. Initial commit `3cb005baf45d364be6f27cc7e7c5b96753cde582`; removal commit Pending.
- [0053: Betaflight bridge document](0053-betaflight-bridge-document.md) — Implemented and verified; upstream FDM/motor reference, three SVGs, 8 focused tests and four motor checks passed; uncommitted, implementation commit Pending.
- [0054: Remove SITL patch](0054-remove-sitl-patch.md) — Implemented and verified; clean upstream build, live pressure/arming/failsafe checks and 20-second hover passed; three baseline test failures documented. Uncommitted, implementation commit Pending.
- [0055: VS Code example picker](0055-vscode-example-picker.md) — Implemented and verified; dropdown JSON and all four example CLIs checked; committed, implementation commit `e8c33198686d7a33c4f224b69602d6d4a8be3240`.
- [0056: Standalone altitude hold](0056-standalone-altitude-hold.md) — Implemented and verified; Godot removed, standalone mission and 44 affected tests passed; committed, implementation commit `e8c33198686d7a33c4f224b69602d6d4a8be3240`.
- [0057: Altitude-hold sequence diagram](0057-altitude-sequence-diagram.md) — Implemented and verified; both README diagrams parsed/rendered, reserved-ID guidance added; committed, implementation commit `e8c33198686d7a33c4f224b69602d6d4a8be3240`.
- [0058: Altitude control explanation](0058-altitude-control-explanation.md) — Implemented and verified; cascade explanation added and all three README diagrams parsed/rendered; committed, implementation commit `e8c33198686d7a33c4f224b69602d6d4a8be3240`.
- [0059: Mission state diagram](0059-mission-state-diagram.md) — Implemented and verified; mission phases/guards documented and all README diagrams parsed/rendered; committed, implementation commit `e8c33198686d7a33c4f224b69602d6d4a8be3240`.
- [0060: Independent Betaflight altitude hold](0060-altitde-hold-bt.md) — Implemented and verified; 11 new checks, 41 regressions and live 10-second SITL hover passed (2.7 cm peak error); committed, implementation commit `73e5b172f77e1b94309ce0cc204a4f2f344d8156`. Workspace lockfile `9064c92bc7ffd8ea62da790396ca3a85e1c2709e`.
- [0061: Betaflight arming diagnostics](0061-bt-arming-diagnostics.md) — Implemented and verified; named blockers, periodic diagnostics, colored states and safe firmware-tail reads; 16 checks and real GUI-mode flight passed; startup stall unconfirmed, implementation commit `73e5b172f77e1b94309ce0cc204a4f2f344d8156`.

- [0062: External renderer package](0062-external-render-package.md) — Implemented and verified; exported/relocated Linux viewer, full automatic flight and owned SHM cleanup passed, plus 83 tests; committed, implementation commit `73e5b172f77e1b94309ce0cc204a4f2f344d8156`. Renderer source `46006ffa7b7dd3858d68c8b8c83e997c0109ca7b`.
- [0063: Betaflight arm-switch recovery](0063-bt-arm-switch-recovery.md) — Implemented and verified; user trace replay, 18 tests and real SITL flight passed; committed, implementation commit `73e5b172f77e1b94309ce0cc204a4f2f344d8156`.
- [0064: Renderer startup methods](0064-renderer-startup-methods.md) — Implemented and verified; separate Godot/ZeroMQ startup, all 15 renderer tests passed; committed, implementation commit `73e5b172f77e1b94309ce0cc204a4f2f344d8156`.
- [0065: Betaflight FDM pacing](0065-bt-fdm-pacing.md) — Implemented and verified; burst regression fixed, 19 checks plus real stalled/headless and GUI flights passed; committed, implementation commit `73e5b172f77e1b94309ce0cc204a4f2f344d8156`.
- [0066: NVIDIA example environment](0066-nvidia-example-task.md) — Implemented and verified; task environment propagation and NVIDIA OpenGL selection confirmed; uncommitted, implementation commit Pending.
- [0067: Position square external renderer](0067-position-square-external-render.md) — Implemented and verified; automatic viewer flight and headless mission passed, 103 affected tests passed (one unrelated missing-layout failure); uncommitted, implementation commit Pending.
- [0068: Square status colors](0068-square-status-colors.md) — Implemented and verified; colored phase transitions, active/settled waypoints, 21 tests and CLI log assertions passed; uncommitted, implementation commit Pending.
- [0069: Godot installation](0069-godot-installation.md) — Implemented and verified; pinned editor, matching export templates, and local verification documented; uncommitted, implementation commit Pending.
- [0070: Betaflight Acro square render](0070-betaflight-acro-square-render.md) — Implemented and verified; independent headless and NVIDIA viewer SITL flights completed; 729 broad-suite passes with two unrelated missing/stale-fixture failures; uncommitted, implementation commit Pending.
- [0071: Acro PID control chain](0071-acro-pid-control-chain.md) — Implemented and verified; Mermaid and SVG rendered; uncommitted, implementation commit Pending.
- [0072: Position square control chain](0072-position-square-control-chain.md) — Implemented and verified; two Mermaid-rendered SVGs separate navigation and mixer stages, with feedback mapped in a table; implementation commit `abc4b70`.
- [0073: External custom URDF loading](0073-external-custom-urdf.md) — Implemented and verified; 64 config, physics, and external-RC tests passed; implementation commit `13cecd004545235fffb78fdfa840e59f7902e361`.
- [0074: PX4 Iris custom profile](0074-px4-iris-custom-profile.md) — Implemented and verified; custom PX4 Iris URDF and runnable YAML profile, with 66 focused regressions passed; implementation commit `13cecd004545235fffb78fdfa840e59f7902e361`.
- [0075: Position square with PX4 Iris profile](0075-position-square-iris-profile-task.md) — Implemented and verified; dedicated VS Code task, complete headless flight and 85 regressions passed; implementation commit `13cecd004545235fffb78fdfa840e59f7902e361`.
- [0076: Custom drone ID renderer compatibility](0076-custom-id-render-compatibility.md) — Implemented and verified; custom `px4_iris` poses map to the renderer slot, and the rendered Iris mission completed; implementation commit `13cecd004545235fffb78fdfa840e59f7902e361`.
- [0077: Render profile and backend title](0077-render-profile-and-backend-title.md) — Implemented and verified; HUD shows profile plus explicit backend mode, renderer rebuilt, 60 Python tests and both Godot checks passed; bullet_drone commit `13cecd004545235fffb78fdfa840e59f7902e361`, bullet_render commit `268d40c4a306450027cd4bcfb60893927975e992`.
- [0078: Complete Iris YAML profile](0078-complete-iris-yaml-profile.md) — Implemented and verified; every config field is explicit and documented, all three focused tests and a full Iris square flight passed; implementation commit `3e54c9c59769ecb568b71099848fa5917abad753`.
- [0079: Custom drone beginner guide](0079-custom-drone-beginner-guide.md) — Implemented and verified; detailed physical calculations, required files, validation, and run commands for adding a quadrotor; implementation commit `3e54c9c59769ecb568b71099848fa5917abad753`.
- [0080: Local custom terrain assets](0080-local-custom-terrain-assets.md) — Implemented and verified; local heightmap and matching collision mesh load from the Iris profile, and the headless square completed; uncommitted, implementation commit Pending.
- [0081: Simulation usage guide](0081-simulation-usage-guide.md) — Beginner walkthrough for running altitude hold and printing per-step ground-truth and sensor observations; implemented and verified, implementation commit `fd94104ab616445eee14e45a47e1ff92a3591f8c`.
- [0082: Altitude-hold observation output](0082-altitude-hold-observation-output.md) — Print ground truth and sensors to stdout at one-second simulation intervals with colored mission-state changes; implemented and verified, implementation commit `f932febf8e273be96dd8aa4cc4dcea0a288e4cd3`.
- [0083: Altitude-hold sensor configuration](0083-altitude-hold-sensor-configuration.md) — Enable rangefinder and IMU-derived attitude estimate readings in the altitude-hold example; implemented and verified, implementation commit `f932febf8e273be96dd8aa4cc4dcea0a288e4cd3`.
- [0084: Altitude-hold render sequence](0084-altitude-hold-render-sequence.md) — Diagram observation creation and selected-pose publication from altitude hold to Godot; implemented and verified, uncommitted, implementation commit Pending.
- [0085: Clickable render flowchart](0085-clickable-render-flowchart.md) — Link render-flow method nodes to their source code in Mermaid; implemented and verified, uncommitted, implementation commit Pending.
- [0086: README examples menu](0086-readme-examples-menu.md) — Add linked summaries for the runnable examples near the top of the root README; implemented and verified, uncommitted, implementation commit Pending.
- [0087: Project purpose overview](0087-project-purpose-overview.md) — Explain the core simulation, observation, control and rendering model at the top of the root README; implemented and verified, uncommitted, implementation commit Pending.
- [0088: Betaflight config comments](0088-betaflight-config-comments.md) — Implemented and verified; each SITL CLI setting is explained and all commands/values remain unchanged; implementation commit `aa3e3079d81f8331412b4f6063ae51b0314bc9ca`.
- [0089: Remove telemetry and recording output](0089-remove-telemetry-recording.md) — Implemented and verified; removed schemas, PlotJuggler sender, and external-RC analysis publication while preserving rendering and sensors; 559 passed, 3 skipped; uncommitted, implementation commit Pending.
- [0090: External renderer interface guide](0090-external-render-interface.md) — Document renderer ownership, transport configuration, topic payloads, image handoff, readiness, and cleanup; implemented and verified, uncommitted, implementation commit Pending.
- [0091: Betaflight SITL terrain following](0091-betaflight-terrain-following.md) — Implemented and verified; independent SITL flight turns +90 degrees CCW and follows the hill to its crest at 3 m rangefinder clearance; implementation commit `abfc209b89962e49f4097139abbac96ee7247fb2`.
- [0092: Smooth Betaflight terrain following](0092-smooth-terrain-following.md) — Implemented and verified; continuous lookahead flight, curved crest U-turn, 8 m offset return lane, and home landing passed focused tests plus headless/rendered SITL; implementation commit `abfc209b89962e49f4097139abbac96ee7247fb2`.

- [0093: Faster terrain following](0093-faster-terrain-following.md) — Increase cruise cap to 3 m/s with 4 m lookahead; implemented and verified, 8 tests and full headless SITL passed, implementation commit Pending.

- [0094: Terrain realtime pacing](0094-terrain-realtime-pacing.md) — Remove blocking status reads from the flight loop; implemented and verified, 86 tests plus headless/rendered flights passed, approximately 0.95 RTF and 28–29 camera FPS, implementation commit Pending.

- [0095: MAVLink terrain monitor](0095-mavlink-terrain.md) — Separate MSP Acro flight with MAVLink monitoring; implemented and verified with tests and headless/rendered flights, implementation commit Pending.

- [0096: MAVLink terrain task](0096-mavlink-terrain-task.md) — Existing VS Code example picker option; implemented and verified, implementation commit Pending.

- [0097: Separate protocol libraries](0097-remove-bullet-msgs.md) — Implemented and verified; 574 repository tests and 111 extracted contract tests passed; uncommitted; implementation commit Pending.

- [0098: Example-owned MSP](0098-remove-msp-link.md) — Implemented and verified; 582 passed, 3 skipped and real headless altitude-hold flight completed; uncommitted; implementation commit Pending.

- [0099: README render diagram](0099-readme-render-diagram.md) — Implemented and verified with Mermaid CLI; uncommitted; implementation commit Pending.

- [0100: Rendered takeoff and image viewer](0100-rendered-takeoff-image-viewer.md) — Implemented and verified; 22 tests and desktop flight passed; uncommitted; implementation commit Pending.

- [0101: Camera viewer FPS](0101-camera-viewer-fps.md) — Implemented and verified; 8 tests and desktop idle check passed; uncommitted; implementation commit Pending.

- [0102: Takeoff viewer task](0102-takeoff-viewer-task.md) — Implemented and verified; uncommitted; implementation commit Pending.

- [0103: Rendered wind controls](0103-rendered-wind-ui.md) — Implemented and verified; 24 tests and desktop wind-update flight passed; uncommitted; implementation commit Pending.

- [0104: Renderer TIME_WAIT restart](0104-renderer-port-time-wait.md) — Implemented and verified; 35 passed, 1 skipped and two consecutive real renderer starts passed; uncommitted; implementation commit Pending.

- [0105: Wind flow diagram](0105-wind-flow-diagram.md) — Implemented and verified with Mermaid CLI; uncommitted; implementation commit Pending.

- [0106: Viewer windsock](0106-viewer-windsock.md) — Implemented and verified; rebuilt viewer, Godot checks, 24 Python checks and live screenshot passed; uncommitted; implementation commit Pending.
