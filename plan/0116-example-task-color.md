# Example task color

## Goal
Give `Bullet: Run example` a green play icon in VS Code.

## Implementation steps
Add the task's `icon` with `id: play` and `color: terminal.ansiGreen`.

## Interface changes
None.

## Verification
Passed: parsed tasks.json and checked the icon and unchanged task label using uv.
VS Code appearance was not checked interactively.

## Scope/defaults
Only this task's icon changes. Use the theme's green color.

## Status
Implemented and verified; uncommitted.

## Implementation commit
Pending.
