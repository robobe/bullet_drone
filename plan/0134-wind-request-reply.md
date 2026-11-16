# Wind request/reply and dataclasses

## Goal and steps
Replace wind topics with validated frozen dataclasses and REQ/REP, using shared transport. Enable only the wind example on optional TCP 5560. Migrate Godot panel/windsock, remove Tk, schema files and publication scheduling. Preserve physics semantics and all other examples.

## Interfaces/defaults
get_wind and set_wind; replies current/applied/rejected plus snapshot. Expected physics/episode and monotonic issue time protect updates. Configuration sets polling 10 Hz, timeout/freshness 1 s. Nonblocking one-request handling between steps; worker owns networking. No mutation retries after timeout.

## Verification
- Python suite: 693 passed, 8 skipped. Focused wind checks: 17 passed.
- Native adapter and live wind pipeline: 26 passed. Godot panel and scene checks passed.
- Packaged renderer rebuilt. A live wind mission applied 0.5 m/s through the Godot panel, updated the windsock, landed and cleaned up.
- Edited Mermaid diagrams parsed and rendered; repository whitespace checks passed.

## Scope
Sibling Godot migration authorized; no unrelated app/recorder changes or lifecycle service. No hard-real-time guarantee.

## Status
Implemented and verified; uncommitted.

## Implementation commit
Pending.
