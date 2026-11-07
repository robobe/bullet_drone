# Smooth Betaflight terrain following

Status: Pending implementation. Implementation commit: Pending.

## Goal

Remove intermediate stops from the terrain-following flight while preserving rangefinder clearance control and the +90 degree heading.

## Implementation steps

1. Replace waypoint settling with lookahead tracking along a sampled continuous route: climb the existing +Y line, take a tangent-continuous 4 m radius U-turn above the crest, descend on an 8 m offset lane, and smoothly merge to home.
2. Cap horizontal speed at 2 m/s, keep existing acceleration and clearance limits, and retain the required hover/settle for landing.
3. Update the terrain-following guide and focused tests to explain and verify continuous passage, the turn, the offset return, and landing.

## Interface changes

No public library interface changes. The terrain-following example's internal route and waypoint-arrival reporting change to represent fly-through progress.

## Verification

Run the focused example tests, Ruff, CLI/task checks, and paced headless and NVIDIA-rendered Betaflight SITL flights. Verify no intermediate waypoint dwell, successful return and landing, and record clearance and path tracking.

## Scope and defaults

Keep the 3 m rangefinder clearance target, +90 degree yaw target, current terrain, vehicle, and 240 Hz simulation. Keep the acceleration cap at 1.5 m/s² and use 2 m/s as a speed cap. Return path is approximately 8 m offset through the crest U-turn and merges toward the original home point. Landing may slow and settle.

## Implementation commit

Pending.
