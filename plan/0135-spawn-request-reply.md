# Spawn request/reply dataclasses

## Goal
Move renderer spawn selection from lifecycle PUB/SUB topics and JSON Schema to the existing shared REQ/REP endpoint. Route operations by an `action` field so wind and spawn share one REP socket and endpoint.

## Implementation steps
- Add frozen request, state and reply dataclasses with strict validation and dictionary export, preserving existing fields and bounds.
- Reuse the configured wind REQ/REP endpoint for spawn and wind; remove `spawn_endpoint` and route requests by operation. Process one request between physics steps while retaining stopped-only replacement semantics.
- Migrate the Godot native adapter and spawn panel to correlated REQ/REP, preserving authoritative status, timeout reconciliation, and UI restrictions.
- Remove spawn schemas and topic registrations; update protocol, renderer and simulation I/O docs.

## Interfaces and defaults
Use the existing optional wind endpoint (`tcp://127.0.0.1:5560` in the wind profile). Include operation names in payloads; no ZeroMQ topics are used for REQ/REP. No automatic mutation retry. Scenario presets and local persistence stay unchanged.

## Verification
Not run by user request; user will verify manually.

## Scope and status
Renderer sibling migration included. Implemented; uncommitted, manual verification pending.

## Implementation commit
Pending.

## Implemented setup lifecycle
The wind example waits stopped for Start on the shared service. Spawn builds and resets a candidate, checks 0.20 m terrain clearance, retains wind settings, then replaces the old simulation. The selected climb is relative to the spawn altitude. Start is routed through the same REQ/REP socket.
