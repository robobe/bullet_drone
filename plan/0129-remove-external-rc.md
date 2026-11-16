# Remove external RC

## Goal and steps
Remove external-RC example, exclusive helpers, receiver/lease and RC wire contract. Prune exclusive publishing and tests, update local docs/tasks, preserve shared render/wind/source contracts and Betaflight.

## Scope and interfaces
Local project only, including protocol package. Remove RC_TOPIC, validate_rc, flight_rc.json and flight_command_endpoint support. Sibling callers require migration; no sibling edits or dependency removals.

## Verification
Passed: project suite 620 passed, 8 skipped; focused protocol/render/wind tests 85 passed including removed-contract rejection and source/JSON validation; Mermaid I/O diagram rendered; local documentation links and whitespace checked. Sibling callers intentionally unchanged. No live flights.

## Status
Implemented and verified; uncommitted.

## Implementation commit
Pending.
