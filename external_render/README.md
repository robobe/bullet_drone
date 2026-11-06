# External Godot renderer

This directory builds a standalone **Ubuntu 24.04/Linux x86_64** viewer from the
existing `bullet_render` Godot scene. The exported package contains the engine,
scene/terrain data, native ZeroMQ extension and non-system native dependencies.
The runtime does not need the Godot editor or the sibling renderer/terrain source
directories. It uses the machine's C/C++ runtime and OpenGL drivers.

## Build

From the `bullet_drone` repository:

```bash
uv run --locked python external_render/build.py
```

Build requirements: the sibling `bullet_render`, `bullet_msgs` and `terrain`
checkouts, CMake/Ninja, a C++ compiler, libzmq development files, Godot 4.7.2 and
its matching Linux export templates. The script uses the existing native CMake
build; it never installs system tools or upgrades pinned dependencies.

The script stages the project temporarily, includes the shared heightmap and
JSON transport profile inside the project data, and exports viewer-only mode.
The existing interactive Godot scene keeps its controls and startup behavior.

The generated package is approximately 85 MiB and is ignored by Git:

```text
dist/linux-x86_64/
├── render                       # executable launcher, paths relative to itself
├── bin/bullet-render.x86_64      # Godot release runtime + embedded project data
├── bin/libzmq_camera.so         # native GDExtension
├── lib/                        # ZeroMQ and non-system dependencies
├── configs/physics.json        # Python camera/publisher profile
├── configs/transport.json      # local TCP endpoints
└── BUILD.txt                   # engine/source version and platform assumptions
```

Copy the **whole** `linux-x86_64` directory to relocate the renderer. Do not copy
only the engine binary: the native extension and bundled libraries are needed.
The launcher supplies its own library search path without replacing the system
C++ runtime, which is also used by graphics drivers. Rebuild after changing the
Godot scene or extension, and stop any owned renderer before replacing its package.

## Run

For a complete automatic flight, use the
[standalone altitude-hold example](../examples/altitude_hold_external_render/README.md):

```bash
uv run --locked --extra render python -m examples.altitude_hold_external_render.main
```

To run just the viewer, which waits for a compatible pose publisher:

```bash
./external_render/dist/linux-x86_64/render
```

The viewer uses its embedded default transport profile. To override endpoints:

```bash
./external_render/dist/linux-x86_64/render -- --transport-config /absolute/path/to/transport.json
```

The example passes its configured transport path automatically. Default ports
are TCP 5555 for poses/config, 5556 for image descriptors and 5557 for the existing
adapter's control/heartbeat endpoint. Viewer-only mode hides simulation controls
and sends no user Start/Stop/Reset or collision commands. The example consumes no
control commands. RGB pixels use POSIX shared memory, so image consumers must run
on the same machine. A working graphics display is required for camera images;
`--headless` is not a rendering substitute.

The Python example removes only its owned renderer's named shared-memory slots
after the child exits, including when a signal bypasses Godot's destructors.

Tracked content is the build recipe, export preset, defaults and documentation.
Generated executables, bundled libraries and editor/import caches stay untracked.
