# Shared workspace environment

## Goal
Use one Python 3.12 uv environment and lockfile for all six sibling Bullet projects.

## Implementation
1. Add a workspace manifest, Python pin, environment setup, and usage guide in the parent directory.
2. Declare sibling dependencies as workspace sources and retire member lockfiles.
3. Configure VS Code build/run tasks and Python selection to use the shared environment.
4. Synchronize all members, extras and development groups; run each project's tests,
   build Python packages, and attempt the native renderer build and headless check.

## Interfaces and defaults
Python package and simulation interfaces remain unchanged. uv discovers the parent
workspace from member directories. Use all packages/extras/groups for shared syncs
and runs to keep every project's tools installed. Native system tools remain external.

## Scope
Environment/build/run configuration and documentation only. No dependency pin upgrades,
system tool installation, commits, or pushes. Missing external viewer tasks are retired; pipeline tooling uses the existing camera/bbox app and shared executables.

## Verification
- Shared locked sync installs 29 packages; all six member directories resolve the same `.venv`.
- `uv pip check` and locked sync `--check` pass.
- All five Python wheels/source distributions build.
- bullet_msgs: 69 passed; los_tracking: 12 passed; mcap_bagger: 65 passed; bullet_app: 11 passed.
- bullet_drone: 593 passed and one task-argument assertion failed in the full run.
  Updated that assertion for the authorized task change; all five profile tests then passed.
- Installed official Godot 4.7 stable in the user-local opt directory, verified
  its archive SHA512, and updated the user-local `godot` symlink; retained 4.3.
- CMake configure/build uses shared Python; extension import now succeeds.
- All 19 renderer tests and the headless frame/FOV check pass under Godot 4.7.
- Real-display scene/pixel check passes on NVIDIA OpenGL. Full live pipeline and
  recording-control runs not performed after installation; their CLI help checks pass.
- Workspace task dependencies/folders, JSON/TOML, Python syntax and shell syntax checked.

## Status
Implemented; shared Python environment, native renderer tests and real-display scene checks verified with Godot 4.7.

## Implementation commit
`b147bb5b4949bba649d178e8c144f15bf845421e` (`proj init`).
