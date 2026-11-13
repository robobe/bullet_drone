# Merge Iris example task

## Goal
Offer Position square (PX4 Iris) in the existing Run example picker.

## Implementation steps
Remove the dedicated Iris task, add its module and profile arguments as a picker
option, and use a shell command to pass the selected arguments separately.

## Interface changes
The dedicated task is replaced by a picker option under Bullet: Run example.

## Scope/defaults
Preserve existing example choices, default, environment, and green icon.
Keep the shared uv extras and frozen lockfile behavior.

## Verification
Passed: JSON parsing, shell syntax and module paths for all 12 choices,
default and icon preservation, Iris profile arguments and path, and git diff --check.
Did not launch the simulator or VS Code task UI.

## Status
Implemented and verified; uncommitted.

## Implementation commit
Pending.
