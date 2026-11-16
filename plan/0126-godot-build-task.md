# Godot renderer build task

## Goal
Expose the existing packaged renderer build in VS Code.

## Steps
Add Bullet: Build Godot renderer using uv run --locked python external_render/build.py, workspace working directory, blue tools icon, default build group and dedicated visible terminal. Document Ctrl+Shift+B in root README.

## Interfaces
New VS Code task only; no application API changes.

## Scope/defaults
Reuse existing script and prerequisite/error handling. No full renderer rebuild or commit requested.

## Verification
Passed: JSON parsed, exact task command/icon/group/working directory checked, existing task preserved, build script and README local links verified. Full renderer build skipped.

## Status
Implemented and verified; uncommitted.

## Implementation commit
Pending.
