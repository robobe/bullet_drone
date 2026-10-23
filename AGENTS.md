# Project guidance

These instructions apply throughout this repository.

## SOLID design guide

- **Single responsibility:** separate CLI parsing, configuration loading, simulation orchestration, environment lifecycle, physics, rendering, and telemetry.
- **Open/closed:** extend physics models and consumers through focused interfaces without rewriting orchestration.
- **Liskov substitution:** implementations must preserve interface units, lifecycle, stepping semantics, and error behavior.
- **Interface segregation:** keep interfaces small; physics consumers must not depend on rendering or plotting operations.
- **Dependency inversion:** orchestration depends on project-owned contracts; concrete library integrations stay in adapters.

Prefer simple composition and small contracts. Introduce abstractions only when they express a real component boundary.

## Constants

- Replace unexplained numeric literals (magic numbers) with descriptive named constants, especially for physical values, PID gains, limits, tolerances, durations, and channel assignments.
- Use `UPPER_SNAKE_CASE` for constant names. Include units where applicable, such as `TARGET_ALTITUDE_M` or `HOLD_DURATION_S`, so each value's meaning is clear.
- Group all module-level constants at the top of the module, immediately after the imports and before classes or functions. Keep constants out of function bodies and avoid scattering them throughout the module.

## Physical effects

- Implement each physical effect in its own named method or focused Python module, with independent verification.
- Separate mixing, actuator dynamics, force calculation, and PyBullet application. Document units, coordinate frames, inputs, outputs, and assumptions.
- Never apply the same effect twice: PyBullet owns gravity/contact, and applying thrust at rotor positions already generates lever-arm torque.
- Track future effects in the [force roadmap](docs/design/rotor-forces-and-hover.md). Treat wind, battery behavior, and motor response as inputs or dynamics rather than standalone forces.

## Libraries and tooling

- Every field in a YAML configuration file must have an adjacent comment explaining its meaning and how to use it. Include units, accepted values or ranges, and default or omission behavior where applicable. Keep these comments current whenever configuration fields change.
- Keep `src/bullet_drone` as a library with caller-owned execution loops. Runnable examples provide their own `main()` entry point, CLI helper, component wiring, pacing, and resource cleanup.
- Use **Click** inside a `CliParser` helper class. The helper returns parsed arguments; it must not launch simulation, load YAML, or configure logging.
- Use **Loguru** for application diagnostics. Configure logging once at each example's application entry point. Library components must not configure global sinks.
- Keep diagnostic logging separate from simulation telemetry recording. Do not use diagnostic messages as the telemetry storage format.
- Use **uv** for dependency management and command execution. Update `pyproject.toml` and `uv.lock` together when dependencies change.
- Keep headless physics independent of optional rendering and plotting dependencies.
- Keep ground-truth `DroneState` independent of noisy sensor readings. Sensor classes live in a separate sensors package and use project-owned data and read-only scene queries; sensor noise must never modify physics state. See the [sensor design](docs/design/sensors.md).

## Workflow and scope

- Read the [design menu](docs/design/index.md) and relevant design documents before changing architecture or interfaces.
- Keep design documents current when interfaces or architecture change.
- Implement only the authorized milestone. Add advanced physics in later steps rather than expanding a basic entry-point change.
- Validate behavior appropriate to the change, including lifecycle and cleanup when changing simulation execution.

## Implementation plans and commit records

- Before starting an authorized implementation, save the agreed plan as `plan/NNNN-short-name.md`. Use increasing four-digit numbers and descriptive filenames.
- Each plan records its goal, implementation steps, interface changes, verification, scope/defaults, status, and implementation commit hash. Use `Pending` until an implementation commit exists; do not invent a hash.
- Maintain [plan/README.md](plan/README.md) as the plan menu. Add a link, short description, status, and implementation commit hash for each plan.
- Keep the saved plan aligned with agreed scope changes, and update its status and verification outcome when implementation is complete.
- After committing an implementation, obtain the actual hash using Git and update it in both the plan and `plan/README.md`. Record additional implementation commits when a plan spans multiple commits.
- Commit these post-commit metadata updates in a separate documentation commit when committing is requested. Record the implementation commit, not the metadata commit; do not amend a commit merely to insert its own hash.
- Saving plans does not authorize implementation, committing, or pushing. In Plan mode, do not write files; persist the plan once execution mode is enabled.

The [entry-point design](docs/design/simulation-entry-point.md) and [ideal rotor milestone](docs/design/rotor-forces-and-hover.md) describe implemented behavior. Later roadmap items do not themselves authorize implementation or dependency installation.
