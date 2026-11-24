# Red truck SVG documentation folder

## Goal

Move the red-truck example's SVG diagrams into `docs/red_truck_visual/` and keep
the README image links working.

## Implementation steps

1. Move every SVG from `examples/red_truck_visual/` into `docs/red_truck_visual/`.
2. Update all repository references to the new paths.
3. Inspect the resulting paths and diff.

## Interface changes

Documentation asset paths change. No code interfaces change.

## Verification

Check that each referenced SVG exists and review `git diff --check`. Tests are
not run.

## Scope and defaults

Only the red-truck example's SVG assets and their references move.

## Status

Implemented; all four README image references resolve and `git diff --check`
passes. Tests were not run. Implementation commit: Pending.
