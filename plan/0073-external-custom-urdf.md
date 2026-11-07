# External custom URDF loading

Status: Implemented and verified. Implementation commit: `13cecd004545235fffb78fdfa840e59f7902e361`.

## Goal

Allow YAML-driven simulations to select an external four-rotor URDF by path,
while preserving the packaged `racer` and `heavy_quad` names.

## Implementation steps

1. Accept the built-in aliases or a URDF path in `DroneSettings`; resolve YAML
   relative paths against the YAML file's directory.
2. Load actuator geometry and constants and initialize PyBullet from the same
   resolved path. Keep the current four-rotor model and mixer contract.
3. Document the URDF requirements and path behavior, including YAML examples.
4. Add tests for aliases, absolute/relative custom paths, invalid/incompatible
   files, and PyBullet initialization with a custom asset.

## Interfaces and scope

`drone.urdf` remains a string. Built-in aliases are unchanged; absolute paths
and paths relative to the YAML file become valid. Direct `DroneSettings` paths
are relative to the process working directory. Custom models must provide
`base_link` mass, `prop0_link` through `prop3_link` inertial origins, root
`properties` (`kf`, `km`, `thrust2weight`), and the existing rotor order/signs.
No CLI changes, arbitrary rotor layouts, or separate actuator configuration.

## Verification

- `tests/test_config.py`, `tests/test_simulation.py`, and
  `tests/test_external_rc_config.py`: 64 passed.
- Coverage includes built-in aliases, YAML-relative/absolute/direct relative
  custom paths, missing/malformed/incompatible models, and PyBullet loading.
- `git diff --check` passed.

## Implementation commit

`13cecd004545235fffb78fdfa840e59f7902e361`
