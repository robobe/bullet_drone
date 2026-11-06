# Prevent FDM catch-up bursts from triggering RC failsafe

Status: Implemented and verified; committed. Implementation commit: `73e5b172f77e1b94309ce0cc204a4f2f344d8156`.

Goal: Fix RXLOSS/failsafe disarming during flight while host MSP RC delivery
continues. Keep upstream firmware and genuine link-loss protection unchanged.

Diagnosis: The loop paces against absolute simulation time, sending catch-up
bursts after GUI/MSP stalls. Pinned SITL sets simRate to simulation timestamp
delta divided by packet arrival delta, then uses it for firmware clocks. A
temporary instrumented run reproduced in-flight disarming after four 40 ms
stalls: actual RC gap max 0.061569 s, firmware simRate peak 1543.21x. The probe
read the owned child's existing simRate symbol, without firmware modifications.

Steps:
1. Add a fake-clock regression at the run_demo/backend.command boundary, injecting
   blocking status work and an observer stall to expose catch-up packet bursts.
2. Pace immediately before backend.command and anchor the next send deadline to
   actual command completion, rather than elapsed simulation time. Slow loops
   run below real time rather than replaying missed deadlines in a burst.
3. Document maximum pacing/RTF behavior, rerun the instrumented reproducer and
   real headless/GUI flight checks, and retain in-flight disarm failure handling.

Interfaces/scope: Example-loop pacing only; no library/firmware edits, increased
RC rate, relaxed failsafe, gain changes, dependencies, commit or push. Maintain
existing physics timestep and pre-arm recovery. Do not touch concurrent renderer
work. Verification and actual limits will be recorded below.

Verification:
- `/tmp/repro_bt_rc_timing.py` reproduced the exact in-flight disarm using short
  stalls during an accelerated clock interval. Before-fix report is saved in
  `/tmp/bt-rc-timing-before.json`: simRate peak 1543.21x, max host RC delivery
  gap 0.061569 s, four injected 40 ms stalls, disarm at about 4.44 simulated s.
- Fake-clock regression runs the actual demo/backend-command boundary, injecting
  15 ms status work and a 40 ms observation stall. Before the fix it recorded
  zero-gap catch-up sends; afterward every send gap is at least 1/240 s within
  the named floating-point tolerance. Normal suite: **19 passed, 2 opt-in live
  cases skipped**.
- The post-fix clock probe peaked at 1.084x, rather than >1500x, stayed armed
  through a 12-second simulation and reached HOLD. Its high-clock-triggered
  stall condition did not fire after the fix; repeated fixed stalls were checked
  separately by the following real-flight test.
- Real headless stress test with repeated 40 ms observation stalls passed:
  **10.000 s hold, 0.0275 m peak altitude error**, landing/disarming and all owned
  process/worker/directory cleanup checks. No sampled armed RXLOSS/FAILSAFE
  statuses. Finished at 37.296 simulated s / 51.473 host s (RTF 0.72x).
  Log: `/tmp/bt-paced-stress-flight.log`.
- Real GUI CLI run passed all mission phases, completed at 36.758 simulated s,
  and logged no armed RXLOSS/FAILSAFE statuses. Log:
  `/tmp/bt-paced-gui-flight.log`.
- README pacing/RTF behavior updated. `uv lock --check` and `git diff --check`
  passed; firmware working tree is unchanged. Full suite and hardware flight
  were not run. Genuine RC loss still aborts the mission; firmware clock scaling
  itself remains upstream code, rather than a project patch.

The user subsequently authorized committing and pushing all pending changes.
