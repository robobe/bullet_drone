# Drone identity

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

`RenderSettings` carries its own `drone_id` for the render stream. Applications
that publish multiple vehicles should use matching IDs in physics and rendering
settings; the standalone examples default to `drone_1`.
