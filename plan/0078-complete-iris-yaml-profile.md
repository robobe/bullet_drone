# Complete Iris YAML profile

Status: Implemented and verified. Implementation commit: `3e54c9c59769ecb568b71099848fa5917abad753`.

## Goal

Make `custom/iris.yaml` explicitly list every supported YAML configuration
parameter while preserving its current simulation behavior.

## Implementation steps

1. Add omitted sections and nested fields using their current loader defaults.
2. Add an adjacent explanation to every YAML field, including units and
   accepted values where applicable.
3. Add a regression check that compares the profile's explicit field tree with
   the complete loaded configuration field tree.
4. Verify the expanded profile retains the Iris model and loads successfully.

## Interfaces and scope

No loader or runtime behavior changes. Preserve the existing Iris URDF, spawn,
internal backend, timestep and duration values; omitted options become explicit
defaults.

## Verification

- `tests/test_custom_iris_profile.py`: 3 passed, including explicit coverage of
  every field produced by the complete `SimulationConfig` dataclass tree.
- An adjacent-comment check passed for every YAML mapping field.
- The full headless Iris position-square mission completed, including landing
  and disarm at simulation time 59.683 s.
- `git diff --check` passed.

## Implementation commit

`3e54c9c59769ecb568b71099848fa5917abad753`
