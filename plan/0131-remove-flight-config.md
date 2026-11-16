# Remove flight configuration contract

## Goal and steps
Remove flight/drone_1/config, its constants, validator, schema registration, schema file and exclusive tests/documentation. Verify schema rejection and retained contracts.

## Interfaces and scope
Local protocol only. Preserve renderer config, feedback, source/image metadata and shared validation helpers. Sibling consumers require migration; no replacement message or dependency changes.

## Verification
Passed: 96 protocol/publisher/renderer tests; removed-schema rejection, retained contracts, local protocol links and whitespace checked. No live flight test.

## Status
Implemented and verified; uncommitted.

## Implementation commit
Pending.
