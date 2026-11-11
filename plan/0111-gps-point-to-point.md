# Standalone GPS point-to-point MAVLink example

## Goal
Fly 400 m over the existing terrain, taking off 10 m above the start and landing at a second GPS coordinate. MAVLink feedback, MSP commands.

## Steps and interfaces
Keep all example helpers local. Reuse core controllers and renderer. Configure GPS firmware, decode position and attitude, follow the route, land using mapped elevation. No controller ground-truth feedback. Check corridor clearance before arming and abort stale telemetry.

## Verification
Unit checks for coordinates, guidance, frames and freshness; full headless SITL and rendered checks.

## Defaults and scope
Origin (32,35,100 m MSL), local endpoints (-200,0) and (200,0), 3 m/s cruise. Existing terrain. No new dependencies or imports from other examples.

## Status
Not implemented. Only the agreed plan has been saved; example implementation and verification remain pending.

Implementation commit: Pending
