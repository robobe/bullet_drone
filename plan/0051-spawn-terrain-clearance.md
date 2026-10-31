# Preflight terrain clearance

## Goal

Reject custom spawn locations that place the vehicle inside or too close to the terrain, while checking terrain contact for airborne vehicles only.

## Implementation

- Expose the minimum terrain clearance of the loaded vehicle from PyBullet through `RcSimulation`.
- Sample the vehicle AABB footprint against the terrain surface and require a 0.20 m safety margin during spawn initialization.
- Report unsafe spawn requests as `terrain_clearance` without replacing the current simulation.
- Keep terrain-contact termination enabled once the vehicle is armed/takeoff is active.

## Verification

- Safe spawn remains accepted.
- Low-clearance and embedded spawns are rejected before Start.
- Airborne terrain contact still stops the mission.
- Existing flat-ground and no-heightmap behavior remains unchanged.

## Status

Implemented; focused tests passed

## Implementation commit

`0f60657`
