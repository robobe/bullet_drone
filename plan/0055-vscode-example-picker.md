# VS Code example picker

Status: Implemented and verified; committed. Implementation commit: `e8c33198686d7a33c4f224b69602d6d4a8be3240`.

Goal: Add a VS Code task that opens a dropdown to select and run an example.

Steps: use a built-in pickString input for the four runnable example modules;
launch the selected module through uv from the project folder; replace the stale
README task list with usage instructions; verify JSON and each selected CLI.

Interfaces: one folder-level task, `Bullet: Run example`, and its example picker.
No library, CLI or workspace launcher changes.

Scope/defaults: default to altitude hold, retain each example's existing display
and lifecycle defaults, enable the existing render extra, and leave Godot/app
startup to existing workflows. No new launcher, dependencies, commit or push.

Verification: JSON parsed, input/default and module paths checked, and all four
selected commands passed `--help` using the task's actual uv arguments.
`git diff --check` passed. VS Code's interactive picker and full graphics/flight
launches were not exercised for this configuration-only change.
