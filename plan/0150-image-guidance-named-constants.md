# Image guidance named constants

## Goal

Replace unexplained numeric literals in the red-truck image-guidance controller
with named constants that describe their role.

## Implementation steps

1. Name the pitch floor, normalized guidance bounds, forward-pitch floor factor,
   and zero-state defaults in `controllers/image.py`.
2. Update the interpolation and pitch formulas to use those names.
3. Inspect the diff and confirm the numeric values are unchanged.

## Interface changes

None.

## Verification

Inspect the final diff. Tests are not run.

## Scope and defaults

Only numeric literals in `controllers/image.py` are replaced. Controller tuning
values remain unchanged.

## Status

Implemented; tests were not run. Implementation commit: Pending.
