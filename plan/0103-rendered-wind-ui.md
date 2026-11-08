# Rendered flight with live wind UI

Goal: new automatic rendered takeoff example with editable wind.
Steps: reuse existing mission/controllers and CLI; add example-owned Tk wind panel; apply validated constant world XYZ wind at step boundaries; publish authoritative simulation/wind_state using existing publisher; document and add task option.
Interfaces: new examples.wind_control.main; no core API or protocol changes. Tk panel runs on caller-owned simulation thread. Close or Escape interrupts flight and releases resources.
Verification: settings validation, applying/publishing readback, UI cleanup, focused existing renderer checks and desktop flight.
Scope/defaults: existing 3 m takeoff/hold/landing, constant wind only, initial calm enabled, no separate image window or Godot-native control ownership changes.
Status: Implemented and verified; uncommitted
Implementation commit: Pending

Verification outcome: 24 wind/UI/external-render checks passed; real NVIDIA/Godot flight applied UI wind X=0.25 m/s, confirmed physics readback, completed landing/disarm at 32.825 simulation seconds and closed resources. CLI help and diff whitespace checks passed. Full suite and Godot-native editable wind controls not tested (unchanged/out of scope).
