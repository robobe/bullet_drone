# Simulation to Betaflight SITL bridge documentation

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

## Goal

Explain the upstream Gazebo-compatible SITL target, starting with a port and
message block diagram, then documenting FDM fields, sensor mappings and returned
motor commands. Distinguish the current partial implementation and patched build.

## Implementation steps

1. Cross-check the encoder, transport, motor adapter, vehicle geometry and pinned
   upstream firmware. Inspect application connections only for overview context.
2. Add a Markdown design document with endpoint and field tables, conversions,
   asynchronous stepping, validation, lifecycle and implementation limitations.
3. Add three accessible, self-contained SVGs: bridge overview, sensor/packet map
   and top-view motor order. Include a Mermaid sequence diagram in the document.
4. Link the document from the design menu and verify links, packet facts, Mermaid
   syntax, SVG XML and rendered drawings. Record verification results here.

## Interfaces and scope/defaults

No runtime, public API, configuration or dependency changes. English text, SI
units and zero-based packet indices. FDM and motors receive detailed coverage;
MSP is context only. No detailed MAVLink/MSP packets or live SITL runs. This work
does not complete plan 0052. No commit or push is included in this request.

## Verification

- Checked packet consumers against pinned upstream Git HEAD, excluding local
  sensor-only edits. Confirmed default listener binding and pressure conversion.
- All 8 existing Betaflight backend tests passed. The headless motor mapping
  tool passed all four motor cases (no live firmware process).
- Assert-based source checks confirmed packet sizes, all 18 documented FDM byte
  offsets, the pressure example and both bundled vehicle geometries/permutations.
- All 20 local document links resolve. All three SVGs parse as XML and include
  accessible titles/descriptions. Mermaid parsed and rendered with the installed
  Markdown-preview Mermaid bundle in headless Chrome.
- Rendered and visually inspected all three SVGs and the sequence diagram;
  corrected a rotor-axis label overlap and checked final arrow directions.
- `git diff --check` passed. No live SITL run or full regression suite was needed
  for this documentation-only change. Upstream integration remains incomplete.
