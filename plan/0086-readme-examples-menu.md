# README examples menu

## Goal

Add a compact menu of runnable examples near the top of the root README.

## Implementation steps

1. Add an `Examples` section after setup guidance.
2. List each runnable example with a short description and link to its detail section or guide.
3. Add the plan to the plan menu.

## Interface changes

None. Documentation only.

## Verification

- Check menu links against README headings and the external-RC guide.
- `git diff --check`.

## Scope and defaults

- Include all six example choices exposed by the workspace task menu.
- Keep existing detailed run instructions below the new overview.

## Status

Implemented and verified; uncommitted.

- The five in-README section anchors resolve to their existing headings.
- The external-RC guide link target exists.
- `git diff --check` passed.

## Implementation commit

Pending
