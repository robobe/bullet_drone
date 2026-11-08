# Rendered takeoff with a separate camera viewer

```bash
uv run --locked python external_render/build.py
uv run --locked --extra render python -m examples.takeoff_image_viewer.main
```

The example reuses the automatic 3 m takeoff, 10 s hold and landing mission.
It owns three processes: Python simulation, Godot renderer and a separate Python
camera viewer. The camera window subscribes to `camera/front/image` descriptors
and copies RGB/BGR pixels from shared memory with `FrameReader`. Padded rows are
handled. Tkinter supplies the window without another Python package dependency;
install your operating system's Python Tk support if it is unavailable. A desktop
display and the built renderer are required.

The window shows display FPS, refreshed once per second using monotonic wall time.
It counts images actually displayed, excluding skipped frames, and shows zero when idle.

The image viewer starts before the mission, displays the latest available frame,
and closes when the mission completes. Closing it early (Escape or window close)
stops the mission with an error and releases renderer, physics and viewer resources.
Ctrl+C also closes all owned processes. The reader does not unlink renderer slots;
the existing renderer adapter owns that cleanup.

`--renderer` and `--render-config` select the executable and camera/transport profile.
To view images from an independently running renderer:

```bash
uv run --locked --extra render python -m examples.takeoff_image_viewer.viewer \
  --render-config external_render/configs/physics.json
```

The standalone viewer waits for frames until closed. Images are displayed, not recorded.
