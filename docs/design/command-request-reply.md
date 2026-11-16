# Simulation commands on the shared service

The wind example owns SimulationService, serving command, state-query, wind and spawn operations through the same `wind_endpoint` (default TCP 5560). The name remains for transport-profile compatibility. One REP worker receives JSON envelopes `{request_id, payload}` and returns `{request_id, status: "ok", payload}`. Correlation belongs to the envelope.

Frozen `SimulationCommand` dataclasses contain action start/stop/reset, expected physics_id, expected episode_id and monotonic issued_ms. `GetSimulationState` contains only action get_simulation_state. `CommandReply` contains applied/current/rejected status, reason and `SimulationStatus`: physics_id, episode_id, state, reason, simulation_time_s and ready_to_start. Dictionaries cross process boundaries. Commands have no schema_version, sequence or controller ownership. command.json and legacy command/heartbeat PUB/SUB are removed.

Start begins a prepared episode. Start while running returns current. Stop ends the run and freezes physics at a step boundary, without resuming later. Start after Stop or completion requires Reset. Reset is accepted only while stopped, reinitializes physics/sensors/mission at the applied spawn with wind and climb settings retained, creates a new episode and stays stopped. Applying a new spawn also creates a prepared episode. Repeated Stop while stopped is a no-op. Completion and mission timeout leave the application available for Reset. Close the renderer or press Ctrl+C to exit.

The application drains at most one operation between steps or while stopped. Replies follow completed transitions. Pacing restarts on each new run. The native renderer serializes exchanges on one REQ socket, gives queued commands priority over readbacks, and keeps command replies separate from wind/spawn readbacks. Mutations are never retried automatically. A timeout displays Outcome unknown. The renderer neither retries the mutation nor queries lifecycle state automatically.

The legacy lifecycle telemetry topic and state.json schema are removed. Collision proposals still use the renderer's PUB endpoint, default TCP 5557. The shared command service is available only in the wind example. Other automatic examples keep their own loops.

Tests and builds were not run by user instruction. Manual checks: Start, reject Reset while running, Stop, reject Start until Reset, Reset to a fresh stopped episode, repeat the mission, completion, unknown outcomes after lost replies, stale IDs and issue times, window-close cleanup.

Renderer update: lifecycle querying, lifecycle caches and get_control_state have been removed. Renderer commands take identity from fresh wind/spawn readbacks, disabling commands when identities conflict or are unavailable. It ignores lifecycle state in historical command replies for control availability. The simulation state-query handler remains for other clients. Legacy state schema/topic and validators are removed from both packages.
