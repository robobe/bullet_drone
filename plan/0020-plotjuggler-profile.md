# Repository-owned PlotJuggler profile

## Goal

Save the verified UDP/JSON embedded-timestamp preferences and launch the ground-truth layout with them from VS Code.

## Implementation steps

1. Add a minimal PlotJuggler4.conf profile template with UDP Server, 127.0.0.1:9870, JSON, embedded timestamp field timestamp, a 120-second buffer and equal XY scale.
2. Add a small standard-library launcher that copies the template into a temporary XDG_CONFIG_HOME, forwards PlotJuggler arguments, waits for the app and cleans up the temporary profile.
3. Update the existing VS Code viewer task and documentation. Preserve existing layout/sender behavior.

## Interface changes

The viewer task invokes the launcher through uv; --layout and other arguments are forwarded to the installed PlotJuggler executable. The saved profile is a checked-in template; each launch starts from it. Normal user preferences and the template are not rewritten by the running viewer. Connecting still requires the native +/OK action.

## Verification

Check profile content, argument/environment forwarding and temporary cleanup; verify native layout restoration with the profile and inspect the UDP timestamp setting. Run focused profile/task checks and git diff --check.

Outcome: six focused checks passed, including saved UDP/parser settings, temporary profile isolation/cleanup, native argument/exit-code forwarding and VS Code task compatibility. The installed PlotJuggler loaded the layout through the new launcher and exited successfully at layout restoration. git diff --check passed. The full physics suite was not rerun for this viewer-only change (plan 0019 previously passed 569 tests).

## Scope/defaults

No automatic UDP connection, simulation changes, dependencies or commits. Use the existing ~/.local/bin/plotjuggler installation. Runtime preference edits are discarded at exit; change the checked-in template for persistent project defaults.

## Status

Complete; committed.

## Implementation commit

`dd15909896b0bb9a3a0590a6025700a9ec938f7d`
