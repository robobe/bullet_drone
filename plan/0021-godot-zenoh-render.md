# Godot / Zenoh rendering integration

Status: Complete; committed. Implementation commit: `715c9fe249ad01795df73e72c0cd3544cfb07a7c`.

Goal: Physics-owned startup camera configuration and simulation-time capture scheduling,
nonblocking Zenoh UDP pose publication, Godot landscape rendering, local SHM images,
and arrival-timestamp MCAP recording.

Steps: add optional example-side publisher and CLI configuration; preserve existing
plain UDP callers; pace headless square flight when rendering; add renderer/bridge
in sibling bullet_render; require SHM for the configured recorder image topic.

Interfaces: render/drone_1/config and render/drone_1/pose JSON; camera/front/image
raw RGB8. Defaults: 640x480, horizontal FOV 90, 30 Hz, centered forward camera.
No synchronized image/pose recording, terrain collisions, or runtime camera resizing.

Verification: scheduling/lifecycle tests, existing square-flight tests, real Zenoh
UDP and image SHM delivery, Godot parse/frame checks, 60-second recording readback.
Results: 578 physics tests, 34 recorder tests and 5 bridge tests passed.
Godot coordinate/projection and real-render scene checks passed (RGB order,
orientation, terrain normals and UI exclusion). A final 60.003-second run recorded
and independently read back 1,763 SHM-backed images and 1,847 UDP pose messages;
measured delivery 29.35 FPS, peak recorder queue 922,050 bytes. The mission finished
and restarted during the run. A separate fault run verified late recorder startup,
physics pause/resume, slow rendering without blocked physics, stale-frame rejection,
bridge restart and Godot restart. Wheel/source builds and packaged schema loading
passed. No exact image/pose synchronization or terrain collision integration was
implemented, as agreed.
