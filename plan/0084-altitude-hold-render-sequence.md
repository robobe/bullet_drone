# Altitude-hold render sequence diagram

## Goal

Document how the external-render altitude-hold example obtains each observation and publishes its selected pose to Godot.

## Implementation steps

1. Add a Mermaid sequence diagram to the example README showing reset, the physics-step observation flow, and the renderer publishing path.
2. Clarify that the render pose packet is a selected subset of the observation and does not carry sensor readings.
3. Add this plan to the plan menu.

## Interface changes

None. Document the existing simulation, adapter and ZeroMQ publisher flow.

## Verification

- Parse and render the edited Mermaid sequence diagram with Mermaid CLI.
- `git diff --check`.

## Scope and defaults

- Update only the altitude-hold external-render README and plan records.
- Describe the existing pose packet fields accurately rather than labeling the packet as full FDM telemetry.

## Status

Implemented and verified; uncommitted.

- Mermaid CLI parsed and rendered the sequence diagram to a temporary SVG.
- `git diff --check` passed.

## Implementation commit

Pending
