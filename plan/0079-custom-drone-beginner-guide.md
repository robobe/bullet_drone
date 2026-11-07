# Custom drone beginner guide

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

## Goal

Turn `custom/README.md` into a step-by-step beginner tutorial for converting a
quadrotor's physical and motor data into a URDF and YAML profile, then running
that vehicle in the simulation.

## Implementation steps

1. Explain the supported four-identical-rotor contract and body/world frames.
2. Teach mass, center of mass, inertia, rotor-arm torque, hover thrust,
   thrust-to-weight, and RPM/SI motor-coefficient calculations from first
   principles, using the existing Iris model as a worked example.
3. Show the files to create, the URDF link/property structure, profile fields,
   path rules, and a validation checklist.
4. Document exact headless, PyBullet GUI, and external-render commands, including
   the current viewer-geometry limitation.
5. Verify the formulas and commands against the current loader and run Iris
   profile validation and a complete headless square flight.

## Interfaces and scope

Documentation only. No runtime, loader, configuration, or renderer behavior
changes. The guide targets the simulator's existing equal-actuator quadrotor
model and retains the PX4 Iris source references and conversions.

## Verification

- `uv run --locked pytest -q tests/test_custom_iris_profile.py` — 3 passed.
- `uv run --locked python -m examples.position_square.main --headless --profile custom/iris.yaml` — full 59.683 s simulated square, landing, and disarm completed.
- Recomputed the Iris RPM coefficient, torque coefficient, maximum thrust, and thrust-to-weight values with `uv`; all match the guide.
- Loaded `custom/iris.yaml` and its relative URDF path, then checked four rotors and hover thrust margin.
- `git diff --check` — passed.

The guide's instructions for a newly measured custom airframe are documented
and cross-checked against the loader. Only the existing Iris airframe was flown.

## Implementation commit

Pending.
