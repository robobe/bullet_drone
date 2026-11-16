# Spawn request/reply

The wind example starts stopped. Godot Scenario edits and local presets supply a draft, Apply replaces the stopped episode, and Start launches the mission. Requests during flight are rejected.

Wind and spawn share `wind_endpoint` (default TCP 5560), one REP socket, and the existing thread-owned ZmqIo worker. REQ/REP uses one JSON envelope `{request_id, payload}` and replies `{request_id, status, payload}`. The payload action routes `get_spawn`, `set_spawn`, `get_wind`, `set_wind`, or `start_simulation`. There are no spawn PUB/SUB topics.

`SpawnRequest` preserves schema_version 2, controller_id, physics_id, episode_id, request_id, issued_ms, scenario_name, XYZ position_m in Bullet metres (Z up), takeoff_height_m, and adds action `set_spawn`. `GetSpawnRequest` contains action `get_spawn`. `SpawnReply` contains current/applied/rejected status, reason and authoritative `SpawnState`. Frozen Python dataclasses validate input and export dictionaries for Godot. JSON Schema files have been removed.

Updates require matching physics/episode IDs and a fresh monotonic issue time. The application handles at most one queued request between steps or during stopped setup. Candidate initialization and 0.20 m terrain clearance checks precede replacing the simulation, retaining wind settings. Takeoff height is relative to the applied spawn altitude. Networking remains in the worker. Apply is sent once, with readback reconciliation after timeout rather than mutation retry.

Manual verification is pending by user request. Rebuild the packaged renderer before running the wind example.
