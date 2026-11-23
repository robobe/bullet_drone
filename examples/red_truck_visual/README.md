# Red truck visual approach

The Betaflight Acro mission takes off to 30 m above the launch ground, rotates
in place while searching the actual Godot camera stream, then requests a 10 m/s
velocity toward the detected truck. Each RGB frame is copied from renderer SHM
and reduced to a red-pixel centroid. Camera FOV/aspect and drone attitude turn
that centroid into a world-frame bearing. Roll/pitch control horizontal velocity;
thrust controls descent along that bearing, capped at 8 m/s.

If the target disappears, the drone brakes forward motion while the
mission holds altitude and yaws to reacquire it. The initial search is limited
to 30 simulated seconds. Search timeout triggers a controlled landing and
disarm. A swept contact report from Godot is the only success condition.

## Run

Build Betaflight and the packaged renderer once:

```bash
tools/setup_betaflight_sitl.sh
uv run --locked python external_render/build.py
```

Then start from the repository root:

```bash
uv run --locked --extra render --extra sitl python -m examples.red_truck_visual.main
```

The example owns PyBullet, Betaflight SITL, the exported Godot process and image
reader. Stop other rendered examples first because they use the same local
transport ports. A working display/GPU is required.

The detector uses a red-dominant RGB/BGR threshold and the centroid of matching
pixels. The image provides steering direction, not distance; forward speed is
set by the 10 m/s bearing-directed velocity request. Contact is detected against
the rendered truck using the renderer’s swept pose check.

Camera polling, SHM copying and red detection run in the vision worker. A bounded
queue holds only the newest detection, including missing-target results. The
flight loop consumes without waiting and rejects stale frame IDs. Polling drains
at most 64 descriptors and copies only the newest frame. NumPy in the render
extra accelerates detection. MSP status polling runs in a separate worker; the flight loop rejects stale
status and propagates polling errors. Physics packets are paced at the send boundary, counting processing within each
interval and avoiding catch-up bursts because SITL derives its clock rate from packet arrival spacing.

Approach tuning: 10 m/s total velocity request, up to 8 m/s descent, 30° tilt
limit, 25° forward pitch and 45°/s pitch slew. The altitude controller permits
8 m/s vertical speed
and 5 m/s² acceleration. Landing remains limited to 1.5 m/s. These are tuning
values, not a guarantee of interception.

Colored STATE messages report each transition. VISION diagnostics run at 10 Hz
wall time and include normalized dx/dy (right/down positive), red pixel count,
measured/target altitude, vertical speed, target pitch, thrust and RC PWM. LOST
means no fresh target is available. Terminal colors require a TTY.

Takeoff requests 4 m/s and slows near 30 m. Tracking permits braking above its
bearing-directed horizontal velocity target. TIMING logs once per second report wall-clock RTF,
FDM packet Hz, scheduled captures, unique SHM frames, and detector completions.
Camera scheduling remains 30 FPS per simulation second; slower-than-real-time
simulation reduces wall-clock camera FPS.

Validation: two rendered SITL runs achieved renderer-confirmed truck contact at
21.89 s and 21.97 s simulation time. Different starting geometry, wind, or camera
latency may require further tuning.
