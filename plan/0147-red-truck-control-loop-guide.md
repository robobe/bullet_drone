# Red truck control-loop guide

## Goal

Document the complete red-truck control path from MAVLink messages and camera
errors through mission guidance, RC commands, Betaflight motor mixing and the
next PyBullet physics step.

## Implementation steps

1. Add an end-to-end data-flow diagram and camera-error visual.
2. Explain telemetry validation, mission phases, image guidance, altitude and
   attitude cascades, RC conversion, MSP delivery, Betaflight mixing and physics.
3. Include concrete `dy` examples for the 20 m and 40 m tuning.
4. Explain the altitude position gain, its units, initial calculation and role
   in converting approach descent speed to an altitude target.
5. Generate and embed SVG versions of the end-to-end loop, camera-error mapping,
   mission phases and altitude-to-throttle cascade.
6. Parse and render every Mermaid block in the example README, then visually
   inspect the generated SVG assets.

## Interface changes

None. Documentation only.

## Verification

- Check documented constants, ports, rates and equations against current code.
- Render all Mermaid blocks successfully.

## Scope and defaults

Only `examples/red_truck_visual/README.md`, its SVG documentation assets and plan
metadata change. Runtime behavior, dependencies and configuration remain
unchanged.

## Status

Implemented. The guide was checked against current ports, rates, gains and
channel mappings. Four Mermaid sources were rendered into embedded SVG assets,
the six remaining Mermaid blocks also rendered successfully, and all four SVGs
were visually inspected. The altitude position-gain equation and worked tuning
examples were checked against the configured `1.5 /s` gain. Wide SVGs use the
document width, while camera and mission visuals use 70% and 45% respectively
to preserve readable document flow. Implementation commit: Pending.
