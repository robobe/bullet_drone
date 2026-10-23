# Acro position square

## Goal
Add a separate own-loop example flying the same settled 3 m square at altitude
3 m, holding home for 10 s, then landing/disarming through Acro RC input.

## Steps and interfaces
- Reuse square mission route/dwell/landing logic with an explicit flight-mode
  choice defaulting to angle. Acro requires an upstream AttitudeController that
  converts velocity-stage tilt targets to bounded body roll/pitch rates.
- Encode Acro frames for every step, including disarmed steps. Heading feedback
  continues producing yaw-rate commands. Collective thrust remains separate.
- New example owns composition, reset/step loop, GUI pacing, CLI and cleanup;
  named defaults and README with sequence/state diagrams mirror the angle example.
- Add README link and GUI/headless VS Code tasks; update mode-composition docs.

## Scope/defaults
Same route, physics and gains, arrival/touchdown criteria, 90 s timeout and GUI
framing as position_square. No Acro self-leveling inside stabilization: the
navigation stage computes rate targets upstream. No dependencies or commits.

## Verification
Full headless Acro mission: check modes, upstream angle-to-rate conversion, rate
PID path, attitude bypass, waypoint/heading/landing and stopped motors. Test
deadline, errors/interrupt cleanup and CLI/import safety. Run full suite and GUI.

## Status
Implemented and verified; uncommitted.

Verification outcome:
- `uv run --locked pytest -q`: 264 passed, 1 skipped (optional telemetry integration).
- New Acro and existing Angle square checks: 31 passed.
- Headless and paced GUI Acro flights completed at 44.500 simulated seconds.
  Arrival times: 5.179, 10.500, 15.817, 21.133 and 26.450 s; land 36.450 s;
  disarm 43.500 s. Final position approximately (0.00152,-0.00180,0.01349) m,
  with motors stopped.
- Every-step check confirms Acro frames, correct upstream attitude feedback/rate
  conversion, bounded RC requests, rate PID torque and mixer motor speeds.
  Internal attitude control is instrumented to fail if called and was never called.
- Waypoint/heading/home hold/landing, exact deadline, early timeout, conversion/
  reset/step failure and interrupt cleanup, import/help safety all pass.
- Six VS Code tasks parse correctly. Static scan found no unused imports.
- README sequence/state diagrams and mode-composition docs added. Updated SVG
  rendered and visually inspected. Dependencies unchanged.

Implementation commit hash: Pending (no commit requested).
