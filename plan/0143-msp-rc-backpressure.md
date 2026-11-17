# Bound outstanding MSP RC packets

## Goal
Prevent rendered SITL slowdown from accumulating wall-clock RC packets ahead of status requests.

## Steps and interface
Wait for each streamed RC acknowledgement before sending another packet, prioritize queued requests, and retain latest-value RC semantics. Apply to common and GPS-local clients. Preserve request deadlines and disconnect cleanup.

## Verification
Reproduced MSP_STATUS timeout using the exact rendered terrain/MAVLink command in `/tmp/terrain-render-diagnosis.log`. Confirmed firmware serial task defaults to 100 simulation-Hz and consumes one MSP packet per invocation, while RC was sent at 50 wall-Hz.

With acknowledgement pacing, the full rendered mission completed in 85.533 simulation seconds / 263 wall seconds with landing, disarming and cleanup (`/tmp/terrain-render-fixed.log`). After adding the missing-ACK deadline and rejection handling, a final 40-second rendered smoke reached 3 m takeoff without MSP errors and cleaned up on intentional SIGINT (`/tmp/terrain-render-final-smoke.log`).

Focused tests: 57 passed, 3 skipped. This includes bounded RC backlog/latest-value delivery and lost-ACK disconnect checks for both clients. A broader run additionally found three existing Acro renderer tests asserting the removed `set_state` API. Whitespace check passed.

## Status
Implemented and verified; uncommitted. Implementation commit: Pending.
