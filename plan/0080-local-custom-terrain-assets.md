# Local custom terrain assets

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

## Goal

Keep the shared terrain heightmap and collision mesh inside `custom/terrain/`
and configure the PX4 Iris profile to load those local assets.

## Implementation steps

1. Copy the existing float32 heightmap and matching OBJ collision mesh into
   `custom/terrain/` without changing their contents.
2. Point `custom/iris.yaml` to the local heightmap; PyBullet discovers the
   adjacent `terrain.obj` by name.
3. Verify profile path resolution, terrain mesh loading, focused profile
   checks, and a headless Iris square flight.

## Interfaces and scope

No runtime behavior changes. Add the two terrain data assets and change only
the Iris profile's terrain path; keep its current 500 m, 121-by-121 geometry.

## Verification

- Both copied assets match the workspace source files byte-for-byte.
- `tests/test_custom_iris_profile.py` and `tests/test_terrain_rangefinder.py` — 6 passed.
- PyBullet reset successfully with the local heightmap and adjacent collision mesh.
- The full headless Iris square completed, landed, and disarmed at 59.683 s simulated time.
- `git diff --check` — passed.

## Implementation commit

Pending.
