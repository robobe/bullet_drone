# GPS control theory documentation

## Goal and steps
Refactor the GPS example README into a tutorial: geographic conversion, distances,
LOS projection/lookahead, speed/braking/ramp, cascaded control, feedback and lifecycle.
Preserve launch commands and operational details. Add geometry and Mermaid diagrams.

## Interfaces and scope
Documentation only. No runtime APIs, dependencies or flight behavior changes.
Explain current ideal GPS, local 1 km model, known terrain and fixed yaw.

## Verification
Cross-check formulas against code, render Mermaid and inspect it, check local links.
No flight rerun required.

## Status
Implemented and verified: code/formula cross-check, local link checks and
whitespace checks passed. Mermaid rendered successfully with Mermaid CLI and
Chrome, and the generated diagram was visually inspected. No flight rerun.

Implementation commit: Pending
