# NVIDIA environment for example launches

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

Goal: Run examples selected by the VS Code task with the project's existing
NVIDIA PRIME/OpenGL environment settings.

Steps: add __NV_PRIME_RENDER_OFFLOAD=1 and __GLX_VENDOR_LIBRARY_NAME=nvidia to
the task environment; document inheritance by PyBullet/exported renderer child
processes and the equivalent terminal invocation; validate JSON and propagation
through the task's uv command.

Interfaces/scope: Task launch environment only, matching bullet_render's existing
run_godot.sh. No Python entry-point, physics, dependency, commit or push changes.

Verification: Task JSON parsed and both variables were verified inside Python
launched through the task's actual uv arguments. With the same environment,
`glxinfo -B` reported NVIDIA Corporation and NVIDIA GeForce RTX 5070 Laptop GPU,
with direct rendering enabled. `git diff --check` passed. No full flight or
interactive task was rerun. This selects graphics offloading; it does not move
Bullet physics to GPU.
