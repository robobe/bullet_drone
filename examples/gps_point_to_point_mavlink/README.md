# GPS point-to-point flight with Betaflight MAVLink feedback

This standalone example takes off **10 m above the start terrain**, follows a
**400 m** line at up to **5 m/s**, and lands at the destination GPS coordinate.
It uses the same terrain and optional external viewer as the other rendered flights.
All mission, MSP, telemetry and SITL helpers live in this example. It imports no
other examples; the core `bullet_drone` package and existing renderer integration
are its project dependencies.

```bash
# Only needed if the GPS/MAVLink binary has not already been built:
bash tools/setup_betaflight_mavlink_sitl.sh --gps

# Headless, using the existing workspace lock without resolving dependencies:
uv run --frozen --extra mavlink python -m examples.gps_point_to_point_mavlink.main --headless

# External renderer:
uv run --frozen --extra mavlink --extra render python -m examples.gps_point_to_point_mavlink.main
```

On this hybrid-GPU workstation, the exported viewer requires the same NVIDIA
offload environment used by the existing VS Code task:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  uv run --frozen --extra mavlink --extra render python -m examples.gps_point_to_point_mavlink.main
```

Stop other SITL examples first. This example owns TCP 5761 (MSP), TCP 5762
(MAVLink), the bridge UDP ports, its firmware process and its temporary EEPROM.
`--sitl-binary`, `--renderer`, and `--render-config` override executable/config paths.
Ctrl+C disarms and releases resources. No commands configure your physical hardware.

The entry point hardcodes a synthetic GPS world origin `(32°, 35°, 100 m MSL)`.
Start is `(31.998941979585357°, 35.001727163237504°)`, local
`(163.205, -117.321)` m. Destination is
`(32.00074562062305°, 34.998061181190316°)`, local `(-183.205, 82.679)` m.
These geographical coordinates place the synthetic terrain; they do not represent
surveyed real-world terrain. The endpoints were selected against the collision
mesh to provide cruise clearance and a gently sloped landing patch. The earlier
west endpoint at `(-200, 0)` was rejected after a real flight slid on its slope.

Preflight queries the actual terrain collision mesh at 0.5 m spacing across a
5 m-wide corridor and requires at least 3 m clearance. It also rejects landing
sites whose center and four points 0.3 m away span more than 0.05 m in height.
With the default route, cruise altitude is approximately 34.29 m world Z
(134.29 m MSL) and checked corridor clearance is 8.41 m. Cruise maintains a fixed
MSL altitude, rather than following terrain at constant clearance. Landing uses
mapped destination height and the vehicle ground offset. This assumes a known
map and ideal simulated GPS.

MAVLink `GPS_RAW_INT` verifies the GPS fix and satellites. `GLOBAL_POSITION_INT`
provides position, raw GPS MSL altitude and NED velocity, converted to local ENU.
`ATTITUDE` provides orientation: the pinned SITL roll/yaw signs match the project
frame, while its published pitch must be negated. Its angular-rate fields are
hardcoded zero and are not used. Position and attitude streams run at 20 Hz.
The mission reuses core controllers and sends Acro RC over MSP; Betaflight closes
the gyro rate loop. MAVLink is feedback, not the command channel.

The mission holds at the start, takes off, uses an 8 m lookahead on the A–B line,
brakes near B, settles within 0.5 m, and descends at up to 0.3 m/s. It disarms
after stable mapped touchdown and waits for firmware disarm confirmation.
No simulation ground truth or direct sensor readings enter its mission controller.
Ground truth drives FDM/rendering and independent outcome metrics only.

Missing or invalid feedback prevents arming. Once telemetry is established,
position/attitude/fix data older than 0.5 s or heartbeat older than 3 s aborts the
simulation, sends disarm and stops SITL. There is no ground-truth fallback. This
abort behavior is for simulation, not a real-aircraft failsafe.

Run focused checks with `uv run --frozen pytest -q tests/test_gps_point_to_point.py`.
The completion log reports independent landing error, cross-track error, cruise
clearance, terrain contact and firmware arm state. For a repeatable full-flight
check (allow several minutes, with no other SITL running):

```bash
BULLET_TEST_GPS_POINT_TO_POINT=1 uv run --frozen --extra mavlink pytest -q tests/test_gps_point_to_point.py
```

The original 3 m/s version was verified with real pinned SITL in headless and rendered modes: about 208.5 s
simulation time, 0.011 m landing error, maximum cross-track error below 0.44 m,
and minimum measured cruise clearance above 9.95 m. Both runs confirmed terrain
contact and disarming. These measurements use the current ideal GPS and terrain.

Status logs show the received global GPS coordinates as `lat` and `lon` in
decimal degrees, and `alt_msl` in metres above mean sea level.
`distance_to_target` is horizontal distance to landing point B in metres, using
the same local tangent frame as navigation. It approaches zero above B, before
descent. `speed` is measured 3D speed in m/s. Global coordinates come directly
from MAVLink before conversion to the local frame. Example format:

```text
cruise GPS lat=31.9995629 deg lon=35.0004652 deg alt_msl=134.29 m distance_to_target=262.30 m speed=4.67 m/s blockers=0
```

Cruise targets 5 m/s with an 8 m lookahead and ramps the velocity command at
1 m/s² to avoid an abrupt acceleration transient. Actual speed depends on the vehicle
and existing controller limits. Existing controller acceleration, braking and landing limits are
unchanged.

The ramped 5 m/s version passed a complete headless SITL flight in 160.262 s
simulation time (about 23% shorter than the original). Actual cruise was around
4.67 m/s; maximum cross-track error was 0.152 m, landing error 0.019 m, and
minimum cruise clearance 9.952 m. Terrain contact and disarming were confirmed.
