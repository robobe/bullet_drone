# GPS point-to-point task option

## Goal
Expose the standalone GPS flight in the existing VS Code Run example picker.

## Steps and interfaces
Add the example module to the pickString options. Use --frozen for this shared
runner because --locked currently fails on the workspace lock mismatch; preserve
the existing dependency lock, extras, GPU environment and other options.

## Verification
Parse the JSON, verify the option points to an existing module, and run its help
with the task arguments. No flight rerun required for a picker/configuration edit.

## Scope and defaults
Existing picker and default selection. No dependency changes or separate task.

## Status
Implemented and verified: JSON parsed, unique existing module option confirmed,
and task-equivalent help command exited successfully.

Implementation commit: Pending
