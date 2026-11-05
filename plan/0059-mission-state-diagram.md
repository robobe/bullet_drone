# Explain altitude-hold mission states and transitions

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

Goal: Make the altitude-hold README's mission diagram show the actual phase
names, transition guards and dwell-timer behavior.

Steps: replace the existing state flowchart with a state diagram, add a compact
state/action/exit-condition table, document PID resets and failure cleanup, then
parse/render the README diagrams and inspect the new state drawing.

Interfaces/scope: documentation only, based on mission.py and demo.py. No runtime,
tuning, dependencies, commit or push changes.

Verification: Compared states, guards, dwell durations and PID resets with
mission.py and failure cleanup with demo.py. All three README Mermaid blocks
parsed/rendered with the installed Mermaid bundle in headless Chrome. Visually
inspected the final state diagram, including explicit phase labels and self-loop
arrows. `git diff --check` passed. Runtime tests were not rerun for this
documentation-only change.
