# Replace patched SITL with upstream firmware

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

## Goal

Use unmodified pinned Betaflight 2026.6.2. Preserve simulated barometer pressure
through the existing inverse-atmosphere encoding into FDM `position_xyz[2]`.

## Implementation

1. Reverse only the known local sensor-only patch, preserving unrelated changes.
2. Remove patch application, `SITL_SENSOR_ONLY` and the patch file. Require a clean
   firmware checkout, keep the pin/binary path and clean old build artifacts.
3. Disable UDP RC, GPS and magnetometer use in the firmware profile while retaining
   MSP, the real IMU estimator, quad-X mixing and existing control settings.
4. Add pressure round-trip and build-workflow regression checks. Update the bridge
   documentation, SVG notes, README and plan 0052's superseded scope.
5. Build and verify clean upstream sources, run focused tests and motor checks,
   then exercise live sensor feedback, arming/failsafe and headless hover.

## Interfaces, scope and defaults

Keep `<18d` FDM and `<4f` motor packets and all public APIs unchanged. Field 16
encodes measured pressure, not `barometer.altitude_m`; field 17 remains populated.
No patched-backend option, new dependencies, MAVLink work, hardware flight,
commit or push. Preserve the existing uncommitted documentation from plan 0053.

## Verification

- Reversed exactly the former patch in the local firmware checkout. The rebuilt
  source is clean at `e0b7bb01b17b21351057e9ead2d1ab39dd44fa16`; the setup script
  performed a clean upstream build at the unchanged launcher binary path.
- `bash -n tools/setup_betaflight_sitl.sh` passed. All 14 focused encoder/setup
  tests passed, including multiple pressure round trips and dirty-checkout
  refusal. The four headless motor-mapping cases and seven sibling MSP adapter
  tests passed.
- Live harness `/tmp/check_upstream_sitl.py` sent the real FDM encoder's packet,
  intentionally replaced field 17 with 70000 Pa, and changed sensor pressure from
  101325 to 101225 Pa through field 16. MSP altitude changed from 0 to 8.34 m,
  proving upstream consumes position Z rather than the legacy pressure slot.
  The sensor's altitude field was deliberately unrelated to pressure.
- Live MSP feature mask confirmed RX_MSP enabled and RX_UDP/GPS disabled. Sensor
  configuration confirmed magnetometer NONE; RAW_IMU reported `(0, 0, 256)` accel
  counts, zero gyro and zero magnetometer at level rest. Arming, nonzero motor
  outputs, RC-loss disarming and zero targets after firmware shutdown/100 ms
  timeout all passed. Report: `/tmp/betaflight-upstream-live.json`.
- Canonical managed check:
  `uv run python ../bullet_render/tools/check_app_flight.py --hover-check --physics-config configs/basic.yaml --report-dir <fresh-directory>`.
  Passed 20.0 s hold at a 20 m relative target, 0.2812 m peak altitude error,
  0.1286 m peak-to-peak and 0.2078 m RMS over 2001 samples. Final state stopped
  and MCAP finalized. Report:
  `/tmp/betaflight-upstream-basic-hover-s8dttnyb/report.json`.
- Full `uv run pytest -q`: **705 passed, 3 failed**. The same three tests failed
  in an untouched HEAD archive: two PlotJuggler tests expect removed task labels;
  `test_sensor_observations_and_noisy_physics_independence` still expects no
  rangefinder despite the enabled basic profile. Baseline log:
  `/tmp/betaflight-upstream-baseline-tests.log`. No unrelated fixes were made.
- An initial hover run with sibling `config/betaflight.yaml` completed its hold but
  failed the helper's peak-error assertion (0.8272 m). Recorded data showed the
  validator used the first TAKEOFF frame at 1.4131 m after spool-up instead of the
  initial 1.0 m observation. Relative to the initial observation, peak error was
  0.4141 m. This reference/sensor-datum limitation remains in the sibling helper;
  no gains or thresholds were changed to obtain the canonical-profile pass.
- Documentation links and SVG XML passed. Rendered and visually inspected the
  two modified SVGs. `git diff --check` passed. No new dependencies or MAVLink
  changes; no hardware flight. Diagnostic harnesses/reports remain outside Git.
