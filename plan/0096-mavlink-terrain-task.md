# MAVLink terrain task

Goal: Add the MAVLink terrain follower to the existing VS Code example picker.
Steps: Remove the separate MAVLink task, add the example picker option, enable the mavlink extra on the shared task, and validate JSON and CLI startup.
Interface changes: New picker option; shared task includes the mavlink extra.
Verification: JSON parsed, unique task labels asserted, exact task CLI with --help passed, git diff --check passed.
Scope/defaults: Rendered Acro flight with existing dedicated firmware; no rebuild or new runtime options.
Status: Implemented and verified; uncommitted.
Implementation commit: Pending.
