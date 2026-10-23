# Live ground-truth position

The reusable [layout](layouts/ground_truth_position.xml) has two panels:

1. **XY trajectory:** world X versus Y in metres.
2. **Altitude:** world Z in metres versus simulation time in seconds.

Native 3D trajectory display is deferred. Both panels consume the physics process's ZeroMQ multipart JSON pose messages.

## Start the viewer

Run VS Code task **Bullet: Plotter (live position)** in the multi-root workspace,
or **Open PlotJuggler ZMQ ground truth layout** when opening `bullet_drone` directly:

```bash
uv run --locked python plotjuggler/launch.py \
  --layout "$PWD/plotjuggler/layouts/ground_truth_position.xml"
```

The launcher uses this machine's existing `~/.local/bin/plotjuggler` installation.
Adapt `PLOTJUGGLER_EXECUTABLE` in `launch.py` on another machine. ZMQ Subscriber and
JSON Parser extensions must be installed; they are present in the verified installation.

The layout restores the plots, selects the Stream tab / ZMQ Subscriber and displays
a 120-second buffer. **It does not automatically connect.** PlotJuggler 4.0 stores
ZMQ/parser preferences outside its layout and requires the streaming start action.
The viewer task loads the [project profile](profile/PlotJuggler/PlotJuggler4.conf),
which preconfigures ZMQ Connect with TCP transport, address `127.0.0.1` and port `5555`, topic
`render/drone_1/pose`, JSON, **Use embedded timestamp field** with
`simulation_time_s`, a 120-second buffer and equal XY scale. It starts from these
settings each time: the launcher copies the template into a temporary
`XDG_CONFIG_HOME` and removes it when PlotJuggler exits. Normal user preferences
and the checked-in template are not modified by the running viewer. Edit the
template to change persistent project defaults; in-app preference changes during
this project session are temporary. Native PlotJuggler CLI options are forwarded
by the launcher unchanged.

Start the source before running the simulation:

1. Select **ZMQ Subscriber** in Sources. If a previously selected source's dialog
   appears despite the restored selection, choose another source and then ZMQ
   Subscriber to refresh this build's internal selection. The project profile normally
   avoids this mismatch by setting ZMQ Subscriber before the layout is loaded.
2. Set the stream buffer to **120 seconds** and confirm the edit with Enter, so
   the full square flight remains visible. Enable the **1:1** toolbar button for
   equal XY axis scale. Both are already set by the project profile.
3. Press the **+** beside the source selector to start/add the stream. Keep mode
   **Connect**, TCP transport, address `127.0.0.1`, port `5555`, topic
   `render/drone_1/pose`, and serialization `json` (preconfigured).
4. **Use embedded timestamp field** is already enabled with field `simulation_time_s`.
   Accept the dialog to start listening. This connection step is still manual.
5. Run workspace task **Bullet: Position flight**. The physics process publishes
   directly through its render configuration; no sender process is needed.

The profile must match `telemetry_endpoint` in
`bullet_render/configs/transport.json` if that endpoint changes.

## Signals and layout compatibility

PlotJuggler's ZMQ source registers incoming messages under **`zmq/data`**. The
ZeroMQ subscription topic filters messages but does not rename that source topic.
JSON fields are:

| Plot | Field(s) under `zmq/data` |
| --- | --- |
| XY | `position_m[0]` against `position_m[1]` |
| Z/time | `position_m[2]`, timestamp from top-level `simulation_time_s` |

Pose version 2 also publishes ground-truth **world velocity in m/s**. Drag
`linear_velocity_m_s[0]`, `linear_velocity_m_s[1]`, and `linear_velocity_m_s[2]`
into plots for Vx, Vy, and Vz (positive Z is upward). These values come directly
from physics; no position derivative is needed. The topic remains
`render/drone_1/pose` on TCP port 5555. Restart the tools after updating/rebuilding
the publisher, shared schema and native renderer together.

Curves are saved as pending intents and bind when the first matching ZMQ fields
arrive. Use auto-fit if your saved view does not frame the incoming data.

PlotJuggler 4.0 refuses to apply a startup layout with an empty data catalog. The
layout therefore loads [layout_bootstrap.csv](layout_bootstrap.csv), containing
one unplotted `layout_bootstrap` value. This is a viewer compatibility fixture,
not simulated telemetry. The plots bind only to the ZMQ fields above. Keep the
CSV beside the layouts folder when relocating this directory.

Before another flight, stop the source and clear its previous data, then start
again (or restart PlotJuggler with this layout). Each flight uses a new episode
ID and simulation time starts at zero; mixing episodes would overlap their curves.

## Saved ZMQ XY/altitude layout

`layouts/xy_alt_zmq.pj4.xml` opens the altitude panel at **0–5 m** with a
**360-second** time span anchored at its saved absolute timestamp. It also restores
a **360-second stream buffer**. Reload the layout to apply the saved values.
This buffer is retained history, not a guaranteed fixed-width scrolling window.
PlotJuggler 4 live auto-fit can override the saved viewport; the XML range does
not lock the altitude axis. The XY panel and saved MCAP source are unchanged.
