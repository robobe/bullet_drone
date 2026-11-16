# Three mutation-only request topics

## Goal and implementation
Expose spawn, wind and command on one request_endpoint (default TCP 5560), using validated dataclass envelopes {topic, drone_id, payload}. Reply only {success:true} or {success:false,reason}. Remove public IDs, client timestamps, getters, applied-state replies and renderer polling. Keep queue tokens internal. Rename request_io configuration, use one-second reply and queue-wait bounds. Worker owns transport and serialization; application handles one operation at a step boundary.

## Semantics
Spawn payload position_m only, stopped-only, applies a fresh stopped run preserving wind. Wind payload contains all current settings and is allowed running/stopped. Command payload action start/stop/reset: Stop ends the run, Reset is stopped-only, completion stays available and Start after Stop/completion requires Reset. Repeated calls are independent, with no deduplication or retries. Expire queued calls based on server receive time; already started operations may complete after client timeout.

## Scope
Wind example first, and renderer request/reply client migrated together. Future simulation-state/wind subscriptions and display design are deferred; do not fabricate readback values. Keep request drafts and show per-call success/failure. Default takeoff height remains application configuration.

## Verification
Python source syntax parsed and affected tests updated. Edited I/O Mermaid diagram parsed/rendered. No tests or builds by user instruction.

## Status
Implemented; uncommitted.

## Implementation commit
Pending.
