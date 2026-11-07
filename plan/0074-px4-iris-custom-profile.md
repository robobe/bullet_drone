# PX4 Iris custom URDF profile

Status: Implemented and verified. Implementation commit: `13cecd004545235fffb78fdfa840e59f7902e361`.

## Goal

Add an external custom URDF and runnable YAML profile based on the published PX4
Iris Gazebo model, exercising the custom-URDF loader.

## Implementation steps

1. Port the Iris mass, inertia, rotor locations, prop envelope and actuator
   constants into the repository's four-rotor URDF contract.
2. Add a complete headless internal-control profile that loads the adjacent URDF.
3. Record source references, unit conversion and rotor-index mapping in the
   custom folder.
4. Test profile loading, derived vehicle parameters and PyBullet reset.

## Interfaces and scope

Add only assets under `custom/` and a regression test. No runtime API or example
CLI changes. The model is a PX4 simulation reference, not a calibrated hardware
model. Rotor links are ordered for this repository's `(+, -, +, -)` reaction
sign convention.

## Verification

- `tests/test_custom_iris_profile.py`, `tests/test_config.py`,
  `tests/test_simulation.py`, and `tests/test_external_rc_config.py`: 66 passed.
- `custom/iris.urdf` parsed as XML.
- `git diff --check` passed.

## Implementation commit

`13cecd004545235fffb78fdfa840e59f7902e361`
