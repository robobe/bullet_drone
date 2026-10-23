# Single workspace launch task

Goal: replace workspace and folder tasks with one task launching bullet_app,
bullet_render, mcap_bagger and bullet_drone.

Steps: clear folder task lists, replace workspace tasks with one Bash launch task,
build the native renderer before launching, and stop remaining services on exit.
Use the documented external-RC physics and mission-enabled app commands.

Interfaces: VS Code task menu only; no runtime interface changes.
Scope/defaults: retain NVIDIA renderer environment and existing shared configs;
exclude PlotJuggler and the standalone video viewer.
Verification: validate JSON and shell syntax, then exercise launch and cleanup
with stub commands without starting graphics or recording a flight.
Status: Implemented; JSON and Bash syntax checks passed. Stub checks verified
all four launches, cleanup after service exit, and cleanup after task termination.
Real graphics/flight launch was not run.
Implementation commit: Pending.
