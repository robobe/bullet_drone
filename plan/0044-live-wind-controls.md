# 0044: Live wind controls and spectator windsock

Goal: Wind toolbar button opens a separate Godot settings panel, with a terrain
windsock visible only to the spectator. Physics owns world mean velocity and
seeded OU gusts, reusing forces/wind.py and existing aerodynamic force paths.

Behavior: live Apply changes mean wind immediately at the next simulation
boundary; stopped updates do not advance physics. Retain gust/RNG history for
strength/tau/vector edits; clear/reseed on seed change or wind/gust disable/enable.
Episodes and confirmed spawn changes keep applied settings and restart gusts.
Stop freezes simulation time. Process restart loads YAML; no GUI file writes.
Support external-RC/LOS only in v1; other examples show unsupported controls.

Interfaces: simulation set_wind/settings snapshot APIs, network-independent wind
model update, simulation/wind_request and simulation/wind_state on existing ports.
Reuse ownership, epoch, expiry, bounded request-id retry/ack patterns. Publish
cached authoritative state at 10 Hz without advancing the stochastic model.

GUI: world XYZ mean velocity and XYZ gust stddev (m/s), enabled/gust toggles,
positive tau seconds, seed, Apply and draft/applied/result displays. Wind button
only toggles the panel. Stop remains usable while requests are pending.
Windsock: procedural 3 m pole/2 m sock beside confirmed spawn, offset +8 m world Y,
on rendered terrain. Downstream orientation from physics mean+gust; calm <0.1 m/s
hangs down. Spectator layer only, no shadows/collisions, stale indicator. No
Godot gust generation or force calculations; one snapshot drives panel and sock.

Verification: wind update/reset/continuity and force coupling, RNG independence
from reads, real TCP request/retry/rejection/ownership, Godot panel and axis/calm/
stale/spawn checks, native build, real-display spectator/FPV exclusion, regression
checks. No Dryden, wind shear, pulse scheduling, persistence or rotor vibration.

Status: Implemented, verification complete.

Verification outcomes:
- 182 focused physics/runner/sensor/render-publication regressions passed; the
  updated spawn transaction checks also pass with applied wind retained through
  candidate failure, successful replacement and reset (14 live-wind/spawn checks).
- All 140 shared-message tests and 83 app regression tests passed.
- Native C++ extension built successfully. All 24 native/pipeline checks passed,
  covering exact topics, identity, malformed/stale data and integer seed round trips. A real
  native Godot-to-external-RC process test confirms stopped Apply at time zero.
- Headless Wind panel/windsock checks cover drafts, acknowledgements, unavailable
  state, six vector directions, calm, spawn offset and spectator layers.
- Real NVIDIA graphics check confirms Wind toolbar behavior and byte-identical
  drone camera images with windsock visible/hidden, including shadow exclusion.
  Existing frame/FOV and real-render scene regression scripts also passed.
  Screenshot: `/tmp/bullet-wind-controls.png`.

Limits: external-RC/LOS workflow only, no long-duration flight/mission benchmark
or vehicle-specific aerodynamic calibration.
Implementation commit: `f292b468846df7e0d3ac076ac98c0f7bc460b907`.

Sibling implementation commits:
- bullet_msgs: `fcf8911b3d004f50389900e9932bb4a34df5a25e`.
- bullet_render: `1e64fb8d1b5ded584683085059e44615c777f9d9`.
