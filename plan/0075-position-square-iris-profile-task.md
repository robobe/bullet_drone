# Position square with PX4 Iris profile

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

## Goal

Add a VS Code task that launches the position-square example using
`custom/iris.yaml` and its external URDF.

## Implementation steps

1. Allow the position-square example to accept an optional simulation profile.
2. Use profile mass, gravity, initial position, rotor model and RC mapping when
   a profile is supplied, keeping current defaults otherwise.
3. Add the profile-backed choice to the existing VS Code example picker.
4. Tune the external-profile navigation path and use the spawn altitude for
   landing so the Iris mission completes on its taller body.
5. Verify CLI parsing, task JSON, affected tests and a full headless flight.

## Interfaces and scope

The example gains an optional `--profile PATH` argument. The task uses the
existing external-render flow. Stock example settings remain the default.

## Verification

- `tests/test_position_square_example.py`, `tests/test_custom_iris_profile.py`,
  `tests/test_config.py`, `tests/test_simulation.py`, and
  `tests/test_external_rc_config.py`: 85 passed.
- VS Code task JSON parsed and `git diff --check` passed.
- Full headless Iris position-square mission completed in simulation time
  59.683 s (3.682 s wall time), including landing and disarm.

## Implementation commit

Pending.
