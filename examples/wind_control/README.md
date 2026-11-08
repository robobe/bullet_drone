# Rendered takeoff with live wind controls

```bash
uv run --locked python external_render/build.py
uv run --locked --extra render python -m examples.wind_control.main
```

Godot shows the flight while a separate Tk window edits constant world X/Y/Z
wind velocity in m/s. Set values and press **Apply wind**. **Enable wind** controls
whether airflow is active. Values must be finite numbers; calm wind is the default.
Tkinter and a desktop display are required. The mission takes off to 3 m, holds
for 10 seconds and lands. It controls altitude, not XY position: horizontal wind
can cause drift, and large wind may prevent mission completion.

The caller pumps the UI and invokes `RcSimulation.set_wind` between physics steps.
The panel displays applied values. Invalid drafts do not change physics. Closing
the control window or pressing Escape interrupts the mission and closes physics,
Godot, UI and publisher resources. Normal completion also closes all resources.

## How the renderer learns about wind

Physics owns wind and its resulting forces. The example publishes authoritative
`simulation/wind_state` through `RenderPublisher.set_wind`; the existing publisher
sends the latest state over ZeroMQ every 0.1 seconds. It includes settings, current
world velocity, episode identity and the application result. The renderer already
subscribes to that topic. Drone movement independently arrives in pose messages.
No wind force is applied by the renderer.

Godot's built-in editable wind panel requires external-RC controller ownership.
This automatic example uses its own Tk panel and publishes with no controller
owner; it does not enable the Godot wind editor or add visual wind effects.

`--renderer` and `--render-config` select the existing renderer executable and profile.
In VS Code, choose **Wind control (rendered takeoff + live UI)** in **Bullet: Run example**.
