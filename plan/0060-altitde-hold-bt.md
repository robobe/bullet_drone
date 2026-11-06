# Independent Betaflight SITL altitude-hold example

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

Goal: Create `examples/altitde_hold_bt` (the requested spelling), with its own
CLI, SITL session, mission, loop and documentation. Climb to 3 m, hold for 10 s,
land, disarm and exit. Betaflight owns stabilization/mixing; PyBullet owns physics.

Steps:
1. Add Click parsing for headless mode and optional firmware-binary path. Require
   the prebuilt pinned binary; do not build firmware implicitly.
2. Own a private firmware working directory/profile, process and MSP connection;
   detect occupied ports before launch and release resources on all exit paths.
3. Feed FDM while waiting for pre-arm readiness and actual arming, then send
   neutral Angle-mode RC and altitude-cascade throttle. Pace physics at 240 Hz,
   including headless runs, because SITL uses host time.
4. Add independent mission phases and thrust-to-PWM mapping. Ground-truth altitude
   and vertical velocity match the original. MSP feedback gates readiness,
   arming/disarming and connection freshness. This is the documented default.
5. Declare the optional sitl/msp_link dependency, update the workspace lockfile,
   add the dropdown choice and document startup, ports and the control chain.

Interfaces/scope: No imports from other examples or application/renderer code.
Reuse project libraries, the FDM/motor adapter and independent MSP package. No
changes to the original example or library control interfaces. Disarm and clean
up on completion, timeout, lost feedback, firmware exit or interruption. Never
terminate another process. No Godot, patches, hardware flight, commit or push.

Verification:
- New suite: **11 passed, 1 opt-in live test skipped** in normal runs. Covers
  phase transitions, dwell interruptions, actual arm/disarm gating, unexpected
  disarming, PWM conversion, import/CLI isolation, missing binary, occupied port,
  startup failure cleanup and loop error/interruption/timeout cleanup.
- `BULLET_RUN_SITL=1 uv run --locked --extra sitl pytest tests/test_altitde_hold_bt.py::test_live_headless_flight_and_cleanup -q -s`:
  **passed** against the real pinned upstream firmware. Took off, held for
  10.000 s with **0.0273 m peak altitude error**, landed and disarmed, then
  completed at 34.117 simulated seconds. Verified process termination, MSP worker
  stop and temporary-directory removal. Log: `/tmp/altitde-hold-bt-live-test.log`.
- Independent CLI headless run completed at 34.600 simulated seconds with final
  Z=0.0135 m. Log: `/tmp/altitde-hold-bt-flight.log`.
- Original altitude-hold, Betaflight bridge/setup and timing regressions:
  **41 passed**. Original example implementation is unchanged.
- Both README diagrams parsed/rendered and were visually inspected. README links,
  VS Code dropdown JSON, `uv lock --check` and `git diff --check` passed.
- Added optional sitl/msp-link dependency in pyproject.toml and updated the shared
  workspace `../uv.lock` (the parent repository owns that file).
- No GUI/manual picker, hardware or full-suite run was performed. No firmware
  tuning/patch changes or imports from other examples were introduced.
