# MAVLink terrain monitor

Goal: Separate terrain example, MSP Acro commands and Betaflight MAVLink monitoring.
Steps: dedicated upstream build/profile, monitor worker, example wiring, tests and live verification.
Interfaces: optional mavlink dependency and separate runnable example. No control via MAVLink.
Defaults: TCP 5762 monitoring, TCP 5761 MSP, 10 s startup heartbeat, 3 s stale warning.
Verification: Dedicated pinned source built with USE_TELEMETRY and USE_TELEMETRY_MAVLINK; source remains clean. Shared-loop suite: 89 passed, 2 skipped; final monitor suite: 4 passed. Ruff, uv lock --check and git diff --check passed. Headless flight completed in 87.358 simulated seconds; final shared-loop NVIDIA rendered flight completed in 86.421 seconds at approximately 0.95 RTF, with received attitude/heartbeat/altitude telemetry, Acro custom_mode 0 and armed/disarmed transitions. Both landed and cleaned up. Render publisher scheduled 2593 frames with zero queue drops; camera image FPS was not separately sampled. Logs: /tmp/mav-flight.log and /tmp/mav-rendered.log. Workspace lockfile ../uv.lock updated alongside pyproject.toml.
Interface details: existing example runner accepts optional sitl_session_factory; base SitlSession accepts profile_path and extra_ports. New example reuses the original mission/controller/loop. MAVLink is monitoring only.
Limits: no ground station forwarding, no fabricated GPS/clearance telemetry, no hardware flight. Full repository suite not run.
Status: Implemented and verified; uncommitted.
Implementation commit: Pending.
