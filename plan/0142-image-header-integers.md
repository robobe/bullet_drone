# Preserve integer pose headers through Godot JSON

## Goal and steps
Fix Python image descriptor rejection of simulation timestamps after the Godot JSON round trip. After native pose validation, duplicate the header and timestamp dictionaries and explicitly convert fid, sec and nanosec to int64_t before descriptor serialization. Preserve values, strict validation, image descriptor version 3 and independent monotonic capture_ns.

## Interfaces and scope
No wire fields or Python acceptance rules change. Native image descriptor serialization and related round-trip checks only. Update the existing probe helper to match its current no-stream-ID event format.

## Verification
Add native descriptor round-trip checks for exact Python integer types with nonzero nanoseconds, plus strict Python header rejection cases. Source review and Python syntax parsing completed; no tests or builds by user instruction. Rebuild renderer before manual Acro square verification.

## Status
Implemented; uncommitted, manual runtime verification pending.

## Implementation commit
Pending.
