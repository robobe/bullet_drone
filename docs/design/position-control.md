# Position and velocity control

Horizontal position control and horizontal velocity control are separate modules.
Both use independent per-axis PIDs; neither imports PyBullet, rendering or RC.
World axes are X/Y horizontal and Z up. Body axes are X forward, Y left, Z up.

## Interfaces

`PositionController(position_pid).velocity(target_position_xy_m, position_xy_m, dt_s)`
returns a tuple of world XY velocity targets in m/s. PID output limits also bound
the vector magnitude, so diagonal requests do not move faster. `last_output`
stores the last completed tuple; `reset()` clears it and PID history.

`VelocityController(mass_kg, max_thrust_n, velocity_pid, max_tilt_rad).update(`
`target_velocity_xy_m_s, velocity_xy_m_s, vertical_thrust_n, measured_yaw_rad, dt_s)`
returns a frozen `VelocityControlOutput`: world `acceleration_xy_m_s2`, ZYX
`roll_rad` and `pitch_rad`, and collective `thrust_n` in body +Z.

The velocity PID outputs m/s², with per-axis acceleration limits. Multiply by
mass to obtain world horizontal forces. Limit their combined magnitude to
`vertical_thrust_n * tan(max_tilt_rad)`, rotate the tilt request using measured
yaw, and calculate collective force including tilt compensation. At zero vertical
thrust, commanded tilt/thrust are zero. Maximum supported tilt is strictly below
60 degrees. Collective clips at motor capacity; clipping can reduce the vertical
force available. PID anti-windup covers PID saturation, not later tilt or mixer
saturation.

Both modules validate the complete request before updating their PIDs. Nonfinite
values, incorrect vector lengths and nonpositive timesteps raise `InvalidCommandError`.
Velocity control additionally rejects negative vertical thrust. Reset clears
history and `last_output`; unsuccessful input validation preserves them.

## Composition and RC

The caller supplies ideal state feedback. `AltitudeController` independently
controls Z and returns desired vertical force. The caller passes that force and
the horizontal velocity target to `VelocityController`, computes a bounded yaw
rate with `heading_rate()`, then calls `RcCodec.encode(mode='angle', ...)`.
This produces the simulator's existing 18 integer RC channels, not joystick
hardware input. RC yaw is a rate in angle mode; heading control is upstream.

`RcSimulation.step()` decodes the RC frame, runs angle P → rate PID → mixer →
ideal motors/propellers → PyBullet, and returns the next immutable observation.
Thrust bypasses attitude/rate PID. The example owns the loop and resource cleanup.
No forces are applied by the position/velocity controllers.

The [Acro square example](../../examples/position_square_acro/README.md) uses the
same square mission with `flight_mode='acro'` and an injected upstream
`AttitudeController`. Desired tilt is converted once to bounded body roll/pitch
rates using measured orientation; heading control still supplies yaw rate. These
rates are encoded as Acro input, so internal stabilization bypasses its attitude
controller and runs rate PID directly. The mode defaults to angle; Acro without
an upstream converter fails configuration validation before simulation startup.

## Migration

The old combined `PositionController.input()` and `PositionControlOutput` are
removed. Construct the position and velocity modules separately, compose altitude
and yaw control at the call site, and encode the resulting angles/thrust through
`RcCodec`. The existing altitude-hold example and simulation interfaces are unchanged.

## Verification

Controller tests check force directions across yaw frames, vector/tilt/thrust
limits, independent PID history, reset and immutable outputs. The
[square example](../../examples/position_square/README.md) checks every control
link during a full headless mission, settled waypoints, held heading, landing,
stopped motors, deadlines and cleanup. Sensors, motor lag and Betaflight are outside
this milestone.
