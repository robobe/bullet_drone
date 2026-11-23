# Red truck visual approach

The Betaflight Acro mission takes off to 30 m above the launch ground, rotates
in place while searching the actual Godot camera stream, then flies forward at
up to 3 m/s while descending. Each accepted RGB frame is copied from the
renderer’s POSIX shared memory and reduced to a red-pixel centroid. Roll trims
horizontal image error; pitch and thrust trim vertical image error while the
forward command and descent continue.

If the target disappears, forward pitch and roll return to neutral while the
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
limited by the existing 3 m/s navigation envelope. Contact is detected against
the rendered truck using the renderer’s swept pose check.

Camera polling, SHM copying and red detection run in the vision worker. A bounded
queue holds only the newest detection, including missing-target results. The
flight loop consumes without waiting and rejects stale frame IDs. Polling drains
at most 64 descriptors and copies only the newest frame. NumPy in the render
extra accelerates detection. MSP status polling runs in a separate worker; the flight loop rejects stale
status and propagates polling errors. Physics packets are paced without catch-up
bursts because SITL derives its clock rate from packet arrival spacing.

Approach tuning: 6 m/s forward speed, 3 m/s descent, 30° tilt limit, 25° forward
pitch and 45°/s pitch slew. The altitude controller permits 3 m/s vertical speed
and 5 m/s² acceleration. Landing remains limited to 1.5 m/s. These are tuning
values, not a guarantee of interception.

Colored STATE messages report each transition. VISION diagnostics run at 10 Hz
wall time and include normalized dx/dy (right/down positive), red pixel count,
measured/target altitude, vertical speed, target pitch, thrust and RC PWM. LOST
means no fresh target is available. Terminal colors require a TTY.
