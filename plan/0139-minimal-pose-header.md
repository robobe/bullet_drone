# Minimal camera pose and image header

## Goal and steps
Replace pose schemas with PoseMessage(header, drone_id, position_m, orientation_xyzw). PoseHeader.fid is simulation step_index and time_stamp is simulation sec/nanosec. Serialize dataclass dictionaries on the existing pose topic, preserving 30 Hz default simulation-time scheduling and stopped refresh. Copy header and drone_id into version-3 image descriptors. Preserve SHM binary layout, monotonic capture_ns and legacy descriptor readers.

## Interfaces and usage
The renderer uses pose for drone/camera transforms. Remove old pose transport/source fields and pose-based collision reporting. Match image readiness by owned renderer session, layout, drone ID and expected initial pose header. Use internal capture tokens, and bind the single scene drone to its first accepted drone_id. Header values may repeat while stopped and reset, so they are not global IDs.

## Verification
Update contract/metadata checks and docs, perform source syntax checks only. No tests or builds by user instruction.

## Scope/defaults
Delete both pose schemas/registrations and obsolete recorder pose selection. Existing full telemetry and commands remain separate. Image descriptor versions 1 and 2 remain readable; version 3 carries header and drone_id without legacy source.

## Status
Implemented; uncommitted, manual verification pending.

## Implementation commit
Pending.
