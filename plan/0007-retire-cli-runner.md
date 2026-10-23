# Retire shared CLI and runner

## Goal
Remove `src/bullet_drone/main.py`, `runner.py`, their dedicated tests and retired
entry-point integration. Keep the library and examples with caller-owned loops.

## Implementation steps
- Remove the shared CLI entry point/runner and their orphaned parser helpers.
- Delete runner-only tests and CLI-only checks. Preserve YAML, physics, motor,
  controller and renderer verification through direct calls and explicit loops.
- Remove the installed bullet-drone console script and obsolete basic-run tasks.
- Update guidance/docs to make example entry points own parsing, wiring, pacing,
  diagnostics and cleanup. Retain optional rendering/telemetry library adapters.
- Refresh uv.lock after project metadata changes and verify the installed command
  is removed by environment synchronization.

## Interface changes
Retire bullet_drone.main, SimulationRunner and shared CliParser/CliOptions.
Remove their unused SimulationEnvironment/CommandProvider protocol annotations
and rendering option helper. ObservationConsumer remains a caller-owned contract.
The bullet-drone command is removed. Both flight example entry points and
RcSimulation/reset/step/close remain unchanged.

## Verification
Run uv sync --locked, full tests, both headless missions and both example help
commands. Scan executable code/docs/tasks for retired imports and commands.

## Scope/defaults
No replacement runner, new CLI, dependencies, commits or pushes. Preserve existing
configuration loading, physics/controller coverage and optional adapters.

## Status
Implemented and verified; uncommitted.

Verification outcome:
- `uv lock` refreshed metadata; `uv sync --locked` rebuilt the editable package
  and removed `.venv/bin/bullet-drone`. No dependency changes were required.
- `uv run --locked pytest -q`: 251 passed, 1 skipped (optional telemetry integration).
- Both headless flight examples completed with unchanged timing and final state:
  altitude hold 23.225 s; position square 44.496 s.
- Dedicated runner tests and CLI help/import checks removed. YAML tests renamed
  to test_config.py. Physics/motor failure and ground-contact checks retained via
  direct reset/step/close; controller and renderer tests no longer depend on runner.
- Static import scan found no unused imports. No current code, tests, docs or
  tasks reference the retired modules/console entry point. Historical plan records
  retain their original descriptions. Four working flight-example tasks remain.
- Rendering composition diagnostics now name library parameters rather than
  retired CLI flags. GUI flights were not repeated; example behavior is unchanged.

Implementation commit hash: Pending (no commit requested).
