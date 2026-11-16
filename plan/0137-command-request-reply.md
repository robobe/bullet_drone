# Complete shared command migration

## Goal and steps
Reconcile Python command contracts and the wind example with the existing native REQ/REP client. Add frozen SimulationCommand, GetSimulationState, SimulationStatus and CommandReply dataclasses. Dispatch start/stop/reset/query alongside wind and spawn. Remove command.json registrations and command/heartbeat publication in both protocol packages. Keep the renderer's existing correlated queues and freshness checks.

## Lifecycle/defaults
Stop ends the run and freezes physics. Reset is stopped-only, retains spawn/wind/climb settings, creates a fresh episode and prepares Start. Completion stays stopped until Reset or window close. One operation per loop boundary, no automatic mutation retries, and configured issue-time limits apply. Shared wind_endpoint remains TCP 5560 by default.

## Verification
Source review and affected test updates only. Do not run tests, builds or flights, per user instruction.

## Scope
Migrate command consumers, remove obsolete schema references and keep docs aligned. Preserve existing unconnected legacy state contracts in sibling bullet_msgs unless a caller requires migration. This work does not port the legacy external-app mission architecture.

## Status
Implemented; manual verification pending, uncommitted.

## Implementation commit
Pending.

## Package compatibility
Sibling bullet_msgs re-exports command dataclasses from bullet_simulation_protocol through an explicit workspace dependency. Both command schemas and registrations are deleted. Workspace lockfile updated with uv lock; no tests or builds run.
