# Wait when the ZeroMQ poller has no input sockets

## Goal and steps
Stop publisher-only ZmqIo workers busy-spinning. Use the stop event's bounded wait when no SUB, REP or active REQ socket exists; preserve normal socket polling otherwise.

## Interface
No wire or public API changes. Maximum idle publication delay remains POLL_MS. At the restored speed, serialized STATUS waits exposed an RC failsafe; permit one outstanding RC packet alongside a non-RC request in both MSP clients. Keep RC acknowledgement pacing and deadlines.

## Verification
Profiled rendered terrain: approximately 8.9 million poll calls in 25 seconds. Final rendered terrain/MAVLink flight completed 86.321 simulation seconds in 92.56 wall seconds (0.93 RTF), with 2590 scheduled poses (about 28/s), landing, disarming and cleanup. Log: `/tmp/terrain-render-no-spin-final.log`. Focused MSP/ZeroMQ tests: 21 passed. Broader renderer tests: 18 passed, one existing fixture failure from a SimpleNamespace missing sensors.

## Status
Implemented and verified; uncommitted. Implementation commit: Pending.
