# External renderer interface guide

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

## Goal

Document the interface between Bullet Drone and the packaged external renderer, including process ownership, configuration, ZeroMQ topics, message contents, image delivery, readiness, and lifecycle.

## Implementation steps

1. Trace the renderer adapter, publisher, control receiver, shared message contracts, and packaged profiles.
2. Add a design document with a message flow diagram and concise transport/topic tables.
3. Link the guide from the design menu and keep the existing rendering overview focused on behavior.

## Interface changes

None. Documentation only.

## Verification

- Check documented fields and directions against the current publisher and renderer adapter.
- Mermaid diagram rendered with Mermaid CLI.
- `git diff --check` passed.

## Scope and defaults

Document the current `external_render` interface and supplied loopback endpoints. Do not modify the sibling renderer project or change wire behavior.

## Implementation commit

Pending.
