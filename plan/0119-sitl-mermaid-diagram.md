# SITL Mermaid diagram

## Goal
Convert the communication diagram in docs/simulation/sitl_fd.md to Mermaid, omitting ports 9001 and 9004 from the diagram.

## Steps and interfaces
Replace the text block with a sequence diagram for simulator, SITL and application, retaining UDP 9003, UDP 9002 and TCP 5761. Documentation only.

## Scope/defaults
Other protocol descriptions stay unchanged. No commit requested.

## Verification
Passed: Mermaid CLI parsed and rendered the sequence diagram to SVG.

## Status
Implemented and verified; uncommitted.

## Implementation commit
Pending.
