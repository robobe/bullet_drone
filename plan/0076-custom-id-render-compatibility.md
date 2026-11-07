# Custom drone ID renderer compatibility

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

## Goal

Allow the external renderer to display custom-profile drones whose simulation
ID differs from its single fixed render slot.

## Implementation steps

1. Map the simulation drone ID to the renderer's required `drone_1` ID only at
   the ZeroMQ pose serialization boundary.
2. Add a regression assertion using `px4_iris` as the simulation ID.
3. Re-run the rendered Iris mission and focused publisher tests.

## Interfaces and scope

Ground-truth and telemetry identity remain profile-defined. Only the
single-drone render pose uses the viewer's established `drone_1` slot.

## Verification

- `tests/test_zmq_render.py`: 8 passed, including a `px4_iris` pose serialized
  to the renderer's `drone_1` slot.
- The rendered Iris position-square mission received camera frames, completed
  the square, landed and disarmed in 59.683 s of simulation time.
- `git diff --check` passed.

## Implementation commit

Pending.
