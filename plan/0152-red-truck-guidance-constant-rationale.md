# Red truck guidance constant rationale

## Goal

Explain how the forward-pitch and vertical image-guidance constants appear in the
control-loop equations and distinguish selected tuning from calculated values.

## Implementation steps

1. Document the source and role of constants in the forward-pitch equation.
2. Document the vertical `dy` gain and how its speed request becomes an altitude
   target through the position gain.
3. Inspect the equations against controller constants.

## Interface changes

None.

## Verification

Compare the documented equations and example arithmetic with the controller
constants. Tests are not run.

## Scope and defaults

Documentation only; no controller tuning changes.

## Status

Implemented; equations and example arithmetic checked against controller
constants. Tests were not run. Implementation commit: Pending.
