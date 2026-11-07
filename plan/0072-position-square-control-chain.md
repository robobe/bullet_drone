# Position square control chain diagram

Status: Implemented and verified. Implementation commit: `abc4b70`.

## Goal

Append a signal-level control diagram to the position_square README, showing
controller inputs and outputs, feedback, angle-mode stabilization, and mixing.

## Implementation steps

1. Trace the implemented command and feedback path from mission targets to the
   mixer.
2. Add separate Mermaid-rendered SVGs for the navigation cascades and the
   command-to-mixer path, then embed them at the end of the example README.
3. Document feedback signal sources separately to keep feedback arrows from
   crossing the control-flow diagrams.
4. Render and visually inspect both diagrams; check the diff.

## Interfaces and scope

Documentation only. No controller behavior, tuning, or APIs change.

## Verification

- Both Mermaid sources rendered to SVG and visually inspected in Chrome at
  document scale. The diagrams separate navigation and command/mixer stages;
  the README table maps every feedback signal to its controller.
- `git diff --check` passed.
- No application tests run because this change only updates documentation and
  its SVG diagram.

## Implementation commit

`abc4b70`.
