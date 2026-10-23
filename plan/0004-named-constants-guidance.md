# Named constants guidance

## Goal
Require meaningful named constants instead of unexplained numeric literals.

## Implementation steps
Add repository guidance for descriptive UPPER_SNAKE_CASE constants, units in
names where applicable, and a grouped constants block immediately after imports.

## Interface changes
None.

## Verification
Read the updated AGENTS.md and confirm the rule specifies naming and placement.

## Scope/defaults
Guidance only; no existing code refactor, dependencies, commits, or pushes.

## Status
Implemented and verified; uncommitted.

Verification outcome: Read the updated Constants section; descriptive names,
units, UPPER_SNAKE_CASE and grouping immediately after imports are specified.
No tests run for this documentation-only change.

Implementation commit hash: Pending (no commit requested).
