# Yaw align before approach

## Goal

Center the detected truck horizontally with yaw while holding altitude and level
attitude, before image-guided pitch and descent begin.

## Implementation steps

1. Add a center phase between search/reacquisition and approach. Use proportional
   yaw rate from `dx`, negative for a target right of center, capped at 0.6 rad/s.
2. Hold current altitude and level roll/pitch during alignment. Require
   `abs(dx) <= 0.08` continuously for 0.3 s before entering approach.
3. Route reacquired targets back through alignment; route target loss during
   alignment through the existing reacquisition phase.
4. Update the README, phase diagram, diagnostics phase colors, and focused
   phase/yaw tests.

## Interface changes

Add `Phase.CENTER`. No public command signature changes.

## Verification

Run focused red-truck tests, lint changed Python modules, render/check the phase
diagram, and run the documented rendered simulation if the local renderer and
firmware are available.

## Scope and defaults

Center using yaw only before approach; retain roll-only lateral correction once
approach begins. Use a yaw gain of 1.5 rad/s per unit `dx`, rate limit 0.6 rad/s,
center tolerance 0.08, and settle time 0.3 s. Pitch and descent stay at zero/hold
until alignment completes.

## Status

Implemented; added the missing `CENTER` diagnostics color after runtime logging
raised `KeyError`, with a regression check for all phase colors. Focused phase
tests: 5 passed; full red-truck file: 20 passed, 3
existing takeoff/MAVLink fixture failures. Ruff and SVG XML parsing passed. The
rendered run reached search but received no shared-memory camera frames, so it
was stopped before centering could be exercised. Implementation commit: Pending.
