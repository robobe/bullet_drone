# Document the altitude control cascade

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

Goal: Add the explained altitude-to-vertical-velocity control loop to the
altitude-hold README, including feedback paths, equations and hover behavior.

Steps: extend the existing feedback/control section without duplicating its
gain table; add a Mermaid feedback diagram; parse/render the README diagrams.

Interfaces and scope: documentation only, matching existing controller and
mission behavior. No code, gains, dependencies, commit or push changes.

Verification: All three README Mermaid blocks parsed and rendered successfully
with the installed Mermaid bundle in headless Chrome. Visually inspected the
new feedback diagram. `git diff --check` passed. Runtime tests were not rerun
for this documentation-only explanation.
