# Betaflight example arming diagnostics and state colors

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

Goal: Explain why altitde_hold_bt waits on the ground and distinguish mission
state changes with colors, without bypassing firmware arming checks.

Steps:
1. Recheck live headless startup and GUI startup, then diagnose only reproduced
   behavior. Request the user's last phase/launch method while working.
2. Add example-local pinned arming-flag decoding and state-transition colors.
3. Report status periodically and on changed blockers: simulation/host timing,
   phase, actual armed state, flags, RC outputs, altitude/speed and motor age.
4. Log firmware/MSP startup and cleanup, include a safe recent firmware-output
   tail on errors, and enable terminal colors at this example's entry point.
5. Add regression checks, document diagnostic output and verify real SITL again.

Scope/interfaces: Logging only unless a reproducible control defect is found.
Keep examples independent and use Loguru diagnostics, not telemetry persistence.
Do not change firmware settings, PID gains or arming guards. No new dependencies,
commit or push. Preserve the uncommitted example from plan 0060.

Verification:
- The unchanged live headless test passed, including actual arming and a
  10-second hold (0.0272 m peak error). The reported startup stall did not
  reproduce. Log: `/tmp/bt-startup-before-logging.log`.
- The first GUI-mode run armed/took off, but aborted on firmware disarm during
  landing. No control behavior was changed. The second, instrumented GUI-mode run
  completed at 34.112 simulated seconds. It showed transient RXLOSS flags despite
  increasing RC delivery counts; the root of the user's startup stall remains
  unconfirmed. Logs: `/tmp/bt-gui-before-logging.log` and
  `/tmp/bt-gui-diagnostics.typescript`.
- Real PTY output confirmed ANSI state colors, startup details, phase transitions,
  periodic/changed-mask diagnostics, disarm and process teardown messages.
- New diagnostic tests failed before implementation, then passed. Current suite:
  **16 passed, 1 opt-in flight skipped**. Checks pinned/unknown flag decoding,
  ANSI state color, blocker/RC/motor diagnostics, final status on mission failure,
  firmware-tail reads that preserve the child's shared output offset, and the
  previous mission/CLI/lifecycle checks.
- `git diff --check` passed. Colors automatically disable for redirected output.
  No tuning, firmware, arming guards or dependencies were changed. No automatic
  retry or bypass of RXLOSS/ARM_SWITCH restrictions was introduced.

Follow-up: the user supplied a trace confirming a latched ARM_SWITCH/NOT_DISARMED
condition after RXLOSS. Its mission recovery fix is recorded in
[plan 0063](0063-bt-arm-switch-recovery.md).
