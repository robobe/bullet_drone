# Red truck controller modules

## Goal

Separate the red-truck example's image, altitude and attitude controller
configuration and behavior into focused Python modules while keeping mission
phase orchestration in `mission.py`.

## Implementation steps

1. Move image-error guidance state and calculations to its own controller module.
2. Move altitude and attitude controller settings into their own modules.
3. Update the mission, demo and existing controller tests to use the modules.
4. Inspect the changed imports and diff.

## Interface changes

Add example-local controller modules. Keep `RedTruckMission` and its constructor
as the mission entry point; keep existing controller constants re-exported from
`mission.py` for current imports.

## Verification

Inspect import references and the final diff. Tests are not run.

## Scope and defaults

Only the red-truck example's controller module organization changes. Tuning and
mission behavior remain the same.

## Status

Implemented; tests were not run. Implementation commit: Pending.
