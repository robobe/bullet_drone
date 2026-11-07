# Render profile and backend title

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

## Goal

Show the actual run profile and whether it uses internal simulation or
Betaflight SITL in the external renderer HUD.

## Implementation steps

1. Display profile name plus explicit `INTERNAL SIM` or `BETAFLIGHT SITL` in
   the Godot HUD; retain `drone_model` for selecting its visual.
2. Pass profile filename and actual backend from YAML-driven examples; use
   stable example names and known backends for fixed examples.
3. Keep the existing profile metadata message schema and verify representative
   internal, Betaflight, and custom-profile launches.
4. Rebuild the packaged Linux renderer used by the VS Code task.

## Interfaces and scope

No CLI or wire-schema change. The profile name comes from the YAML filename
without extension, or the example name when no YAML profile is supplied.

## Verification

- Focused Python renderer/example tests: 60 passed, 1 skipped.
- Godot `test_scene.gd` and headless `test_frames.gd` passed.
- Rebuilt `external_render/dist/linux-x86_64/render` with Godot 4.7.2.
- Live renderer startup received camera frames with both Iris/internal and
  Acro/Betaflight profile metadata.
- `git diff --check` passed in both repositories.

## Implementation commit

Pending.
