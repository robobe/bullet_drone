# Shared terrain heightmap and rangefinder

## Goal

Export the Godot terrain as a shared heightmap plus deterministic triangle mesh, load the mesh as the PyBullet collision surface, and publish a body-down AGL rangefinder reading in centimetres.

## Implementation steps

1. Add a deterministic heightmap exporter and shared float32 heightmap manifest.
2. Add validated terrain configuration and replace the default plane with the generated terrain mesh when configured.
3. Add arbitrary-ray surface queries and a rangefinder sensor wired into observations and telemetry.
4. Add terrain-contact termination and update schemas, fixtures, and focused tests.

## Interface changes

- Add optional `terrain` configuration for heightmap path, world size, sample dimensions, and origin.
- Add optional `sensors.rangefinder` configuration for enablement, maximum range, and body mount offset.
- Replace the telemetry `rangefinder: null` placeholder with a nullable reading containing timestamp identity, distance in centimetres, and validity.

## Verification

- Verify exporter samples match Godot terrain values.
- Verify heightmap dimensions, generated-mesh collision, body-down raycasts, no-return validity, centimetre conversion, and terrain contact termination.
- Verify strict telemetry schemas and older packets remain valid.

## Scope/defaults

- Keep the current 500 m terrain as one heightfield; defer tiling.
- Use PyBullet as the authoritative collision and sensor source; Godot consumes the same asset for rendering.

## Status

Pending

## Implementation commit

Pending
