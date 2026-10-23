# Simulation telemetry schemas and configuration

## Goal

Replace the MCAP adapter with ten transport-independent simulation topic schemas, a shared header schema, and YAML selection of optional topics.

## Implementation steps

1. Create one module per topic exporting TOPIC and SCHEMA; share a required header containing schema version, drone ID, episode ID, step index and simulation time.
2. Define mandatory ground_truth/config/events and optional sensors/rc_input/control/allocation/forces/airflow/timing topics. Control, forces and airflow also identify their source step/time.
3. Add strict boolean TelemetrySettings fields (all optional topics default true), YAML loading/comments and a pure registry selection helper.
4. Remove the MCAP adapter and dedicated tests; retain physics/sensor identity coverage with transport-independent payload tests.
5. Document units, frames, schema usage and the future centralized recorder/performance strategy. Update the design menu and legacy telemetry instructions.

## Interface changes

Samples use {header, data}; schemas are self-contained JSON Schema documents. SimulationConfig gains telemetry settings. The topics package exports a topic-to-schema registry and selection helper. Legacy MCAP APIs and physics/* topic names are removed without aliases.

## Verification

Validate all schemas and representative JSON payloads, shared header requirements, source timestamps, malformed payload rejection, topic selection and strict Python/YAML settings. Preserve live drone identity and sensor payload checks, verify import safety, and run the full suite plus git diff --check.

Outcome: focused checks passed (219 tests), the full suite passed (528 tests), and git diff --check passed. Import safety also passed after asserting that production imports do not load jsonschema. jsonschema is a development-only dependency, recorded in pyproject.toml and uv.lock; no transport dependencies were added. No GUI run or recorder performance benchmark was needed because this milestone changes no runtime collection or simulation execution.

## Scope and defaults

Schemas/configuration only: no runtime collection, queues, workers or file/network I/O. Mandatory topics cannot be disabled; optional flags default true. Frames use world Z up, body-to-world XYZW quaternions and prop0..prop3 rotor order. Git commit may be null. Simulation events exclude mission phases. Future recorder work stays in docs/todo.md.

## Status

Complete; committed.

## Implementation commit

`0e629400438b6f1d2fcbb4e37b40853ed3108aa7`
