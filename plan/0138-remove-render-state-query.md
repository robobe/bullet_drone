# Remove legacy lifecycle schema and renderer state polling

## Goal and steps
Delete state.json, its registrations/validators/topic publication and consumer references in both protocol packages. Replace local set_state scheduling with set_running(bool). Remove renderer lifecycle query scheduling, caches and get_control_state API. Keep command buttons with fresh wind/spawn identities and correlated replies. Let simulation enforce lifecycle restrictions.

## Interfaces/defaults
Shared endpoint retains commands/wind/spawn. Simulation get_simulation_state handler remains available to other clients, but renderer never sends it. A command timeout displays Outcome unknown, releases pending state, and never retries or queries lifecycle automatically. Conflicting fresh identities disable commands. Pose/source age and sequence checks protect captures, collisions and health reporting without lifecycle gates. Local stopped refresh remains 1 Hz.

## Scope
No replacement lifecycle synchronization. Update probes, app legacy subscriptions, recorder entries, tests and docs. Preserve unrelated spawn/wind state data and the SimulationStatus command reply.

## Verification
Source review and Python syntax parsing completed. Updated I/O Mermaid block parsed and rendered. Do not run tests or builds. Manual runtime verification pending.

## Status
Implemented; uncommitted.

## Implementation commit
Pending.
