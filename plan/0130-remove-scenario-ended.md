# Remove scenario end message

## Goal and steps
Remove scenario/ended, ScenarioEnd, validate_end and schema; remove its embedded terminal_result lifecycle extension. Preserve ordinary v1 lifecycle state and collision contracts. Update tests/docs.

## Interfaces and scope
Local protocol only; sibling consumers intentionally unchanged as in prior removal. No replacement topic.

## Verification
Passed: 97 protocol/publisher/renderer tests; source/docs search confirms removed identifiers absent; whitespace checked. No live flight.

## Status
Implemented and verified; uncommitted.

## Implementation commit
Pending.
