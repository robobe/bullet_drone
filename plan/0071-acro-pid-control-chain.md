# Acro PID control chain

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

## Goal

Explain the position_square_acro controller cascade, PID ownership, signal units,
and feedback paths in its README, with a Mermaid diagram and a standalone SVG.

## Implementation steps

1. Document the outer XY and altitude cascades, external attitude-to-rate mapping,
   MSP Acro encoding, and Betaflight gyro rate PID/motor loop.
2. Add a readable SVG depicting both cascades, the firmware boundary, and feedback.
3. Render-check the Mermaid and SVG and record verification here.

## Interfaces and scope

Documentation only. No controller behavior, gains, APIs, or dependencies change.

## Verification

- Mermaid control chain rendered successfully with Mermaid CLI.
- SVG parsed as XML and rendered in Chrome at its 1500×970 viewBox; visual layout
  inspected. `git diff --check` passed.
- No application tests run because this change only updates documentation and
  diagrams.

## Implementation commit

Pending.
