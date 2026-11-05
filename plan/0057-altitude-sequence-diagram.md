# Repair altitude-hold sequence diagram

Status: Implemented and verified; committed. Implementation commit: `e8c33198686d7a33c4f224b69602d6d4a8be3240`.

Goal: Fix the sequence diagram in the altitude-hold README and prevent the same
Mermaid syntax mistake through repository guidance.

Steps: reproduce the parser failure; replace reserved participant ID `Loop` with
`DemoLoop` while retaining its display label; add Mermaid participant-ID and
validation guidance to AGENTS.md; parse and render both README diagrams.

Interfaces: documentation only. Preserve diagram behavior and all application
code. No dependencies, commit or push.

Verification: The installed Mermaid parser rejects the original sequence with
`Expecting '+', '-', '()', 'ACTOR', got 'loop'`. Flowchart already parses.
After renaming the ID, both README diagrams parsed and rendered successfully
using the installed Mermaid bundle in headless Chrome. Visually inspected the
repaired sequence diagram. `git diff --check` passed. No runtime tests were run
for this documentation-only fix.
