# Recover a failed Betaflight arm request

Status: Implemented and verified; committed. Implementation commit: `73e5b172f77e1b94309ce0cc204a4f2f344d8156`.

Goal: Recover the supplied ARMING trace: RXLOSS followed by NOT_DISARMED and
ARM_SWITCH while the example holds AUX1 high indefinitely.

Steps: reproduce the supplied flag sequence in a mission regression; return
unarmed ARMING attempts with blockers to PREARM and send arm/throttle low; require
0.5 s continuously disarmed with no blockers before requesting arming again;
document the recovery edge and verify unit and live SITL behavior.

Interfaces/scope: Existing phases/RC channels, no firmware or gain changes. Keep
physics held during pre-arm recovery. Recovery applies before takeoff only;
unexpected disarming during flight remains fatal. No bypass of firmware checks,
new dependencies, commit or push. The source of transient RXLOSS is separate.

Verification:
- Replayed the exact user masks `0x2000000c` then `0x20000008`. The test failed
  before the fix because the mission remained ARMING with AUX1 high. It now
  verifies immediate arm/throttle=1000, stable-clear dwell, interruption/reset of
  that dwell and successful takeoff after actual arming confirmation.
- Additional test checks that PREARM waits for actual disarming before starting
  its readiness dwell. Existing unexpected in-flight disarm test remains intact.
- Normal new-example suite: **18 passed, 1 opt-in live test skipped**.
- Real upstream headless flight: **passed**, 10.004 s hold with 0.0277 m peak
  altitude error, successful landing/disarming and teardown at 36.904 simulated
  seconds. Log: `/tmp/bt-arm-recovery-live-final.log`.
- The first live run completed normally but exposed floating-point roundoff at
  the test's one-tick hold-duration upper bound. Added a named 1 ns comparison
  tolerance to that test only, then reran successfully. No flight tolerance or
  controller gains changed.
- Both README diagrams parsed/rendered and the recovery edge was visually
  inspected. `git diff --check` passed.
- No new GUI-mode run or induced live RX loss was performed. The supplied trace
  is replayed deterministically at the mission boundary; intermittent RC-loss
  generation remains separate. Persistent RXLOSS still prevents arming.

The user subsequently authorized committing and pushing all pending changes.
