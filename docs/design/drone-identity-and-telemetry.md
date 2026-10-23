# Drone identity and telemetry

Set stable identity under the drone configuration:

```yaml
# Required vehicle settings.
drone:
  # Optional stable identity; nonblank, up to 128 characters; default drone_1.
  drone_id: survey-drone-2
  # Required positive total mass in kg.
  mass_kg: 1.0
  # Required finite world XYZ spawn position in metres, Z up.
  initial_position_m: [0.0, 0.0, 1.0]
```

`DroneSettings.drone_id` defaults to `drone_1` when omitted, preserving existing
YAML and positional constructors. IDs must be strings with at least one
non-whitespace character and at most 128 characters. Values are preserved,
including any surrounding whitespace; the loader does not silently rename them.
Callers choose distinct IDs when distinguishing multiple drones.

Every physics `DroneState` has the configured `drone_id`; it is stable through
steps and resets. `episode_id` remains a new UUID on each reset. They identify
different things: the vehicle and the simulation run. The state field is appended
with the same default, preserving existing positional construction and immutable
observation behavior. Synthetic states should use an ID valid under the same rules.

Every [simulation telemetry topic](telemetry.md) requires a shared header with
`schema_version`, `drone_id`, `episode_id`, `step_index` and `simulation_time_s`.
The payload is under `data`; vehicle identity belongs in the header rather than
being repeated in each topic's payload. When constructing a sample from an
observation, take identity/time from that observation's ground-truth state.

The schemas are transport-independent and self-contained. The old MCAP adapter
and `physics/*` topic names are removed without compatibility aliases. Historical
files retain their embedded schemas; no recordings are rewritten. New schemas
and YAML switches alone do not collect, store or transmit observations.

Rendering profiles currently carry their own `RenderSettings.drone_id`; this
change does not alter rendering configuration or packet identity. Applications
composing rendering and recording should configure the same identity for both.
The standalone examples construct settings directly and inherit `drone_1`.

Config, live physics, sensor payload and shared-header schema tests run without
any recording service. Schemas are validated using the development-only
`jsonschema` dependency; simulation code does not import it.
