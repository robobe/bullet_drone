# Red truck camera approach with MAVLink feedback

The mission reads Betaflight attitude and barometer altitude over MAVLink on TCP
5762. A dedicated worker owns the connection and publishes a capacity-one queue.
Each sensor has an independent timestamp, with a 0.5 s freshness limit. Missing,
invalid or stale telemetry stops the mission. There is no ground-truth fallback.

PyBullet truth remains in simulation, rendering, frame IDs and contact validation.
It is not passed into the mission controller. MSP on TCP 5761 carries arm status
and Acro RC commands. The package owns its SITL wrapper and firmware profile.

## Run

```bash
bash tools/setup_betaflight_mavlink_sitl.sh
uv run --locked --extra render --extra mavlink python -m examples.red_truck_visual.main
```

Requires the packaged Godot renderer and an available display. Default firmware:
`bin/betaflight_mavlink_SITL.elf`. GPS is disabled. The MAVLink build must support
ATTITUDE and VFR_HUD. Pinned firmware publishes zero climb and angular-rate
fields, so climb is estimated from altitude changes and angular-rate fields are
not used by the outer controller. Betaflight closes its gyro loop internally.

## Camera steering

Forward pitch increases toward 22 degrees when horizontal camera error is small,
and decreases as horizontal error grows. Vertical error adds pitch correction.
Final pitch stays between 5 and 30 degrees and slews at 40 degrees/s. Roll combines 12 degrees times normalized horizontal error with a filtered
30 degree-second error-rate term, limited to 30 degrees. This is an
image-based pitch heuristic: no horizontal speed is measured or controlled.
Maximum physical forward velocity cannot be established from these sensors.

Positive dx means target right, positive dy means target down. On target loss,
the mission levels pitch and yaws to reacquire. Leveling does not guarantee
horizontal braking without a horizontal motion estimate.

## Detailed control loop

The controller is a cascade rather than a direct world-position controller.
MAVLink supplies measured attitude and altitude, the camera supplies target
error, the mission produces RC commands, and Betaflight closes the fast gyro and
motor loop. The mission never receives the simulated world position or velocity.

<p align="center">
  <img src="control-loop.svg" width="100%" alt="Closed control loop from camera and MAVLink feedback through mission RC commands, Betaflight motor outputs and PyBullet physics.">
</p>

### Camera error convention

The detector reports the red-pixel centroid relative to the image center. Both
axes are normalized, so the center is `(0, 0)` and the approximate image edges
are `-1` and `+1`.

```text
                         dy = -1
                    target above center
                            ↑
                            │
        dx = -1  ←──────── (0,0) ────────→  dx = +1
       target left         image center       target right
                            │
                            ↓
                    target below center
                         dy = +1
```

<p align="center">
  <img src="camera-error-control.svg" width="70%" alt="Camera dx and dy errors mapped to roll, forward pitch, pitch correction and vertical-speed correction.">
</p>

- Positive `dx` means the truck is right of center; negative `dx` means left.
- Positive `dy` means the truck is below center and more descent is needed.
- Negative `dy` means the truck is above center, so descent is reduced and can
  reverse into a climb.
- `dx` affects lateral roll and reduces forward pitch when the target is far
  from the horizontal center. `dy` affects both pitch and vertical thrust.

### 1. MAVLink input and validation

`SensorWorker` connects to `tcp:127.0.0.1:5762` and accepts only `ATTITUDE` and
`VFR_HUD` messages. `ATTITUDE` roll, pitch and yaw are converted to a quaternion;
the pitch sign is changed to match the project coordinate convention. `VFR_HUD`
provides barometric altitude in meters.

The pinned firmware does not provide useful climb data, so vertical speed is
estimated from successive altitude messages and low-pass filtered:

```text
raw vertical speed = (altitude now - altitude previous) / elapsed time
filtered speed += elapsed / (0.2 + elapsed) * (raw speed - filtered speed)
```

Attitude and altitude have independent receive timestamps. A combined
`SensorSample` is timestamped with the older of the two, ensuring that a fresh
attitude message cannot hide stale altitude. Non-finite values fail the worker,
and a sample older than 0.5 seconds stops the mission. A capacity-one queue
keeps only the newest complete sample.

### 2. Inputs collected by the flight loop

The caller-owned loop runs at 240 Hz. Every iteration collects:

- arm state and arming blockers from MSP status;
- MAVLink attitude, altitude and filtered vertical speed;
- the latest fresh camera `dx` and `dy`;
- renderer-confirmed truck contact; and
- simulation time and step index for lifecycle and freshness checks.

These inputs are passed to `RedTruckMission.command()`. Ground-truth pose is used
only by physics, rendering, frame matching and contact validation; it is not a
mission-control input.

### 3. Mission phase selects the targets

<p align="center">
  <img src="mission-phases.svg" width="45%" alt="Mission phases from prearm and takeoff through search, approach, reacquisition, landing and completion.">
</p>

The phase determines the outer-loop targets:

- **Takeoff:** hold level and climb to the configured altitude, limited to
  4 m/s.
- **Search:** hold the measured altitude and rotate at 0.35 rad/s.
- **Approach:** use `dx` and `dy` for roll, pitch and vertical guidance.
- **Reacquire:** slew pitch back to level, hold measured altitude and rotate at
  0.25 rad/s.
- **Land:** target ground altitude with descent limited to 1.5 m/s.

### 4. Image guidance produces roll, pitch and altitude targets

During approach, horizontal image error produces a roll target:

```text
roll target = 12 deg * dx + 30 deg-s * filtered change of dx
```

The derivative term damps rapid lateral movement in the image. Roll is limited
to 30 degrees.

Forward pitch combines a baseline and the vertical image correction:

```text
forward baseline = 22 deg * max(0.2, 1 - abs(dx))
pitch target = forward baseline + 10 deg * dy * altitude scale
```

The altitude scale is 0.5 at 20 m and below, rises linearly, and reaches 1.0 at
40 m. Thus `dy` adds half as much pitch at 20 m as at 40 m. Final pitch is
limited to 5–30 degrees and slew-limited to 40 degrees/s.

Vertical guidance deliberately keeps full `dy` authority at both altitudes:

```text
descent request = base descent + 4 * dy
```

Base descent is 3.0 m/s at 20 m and below, rises linearly, and reaches 3.5 m/s
at 40 m. Positive descent means down. For a 20 m approach:

| `dy` | Calculation | Requested vertical motion |
|---:|---:|---|
| `+0.5` | `3.0 + 4 × 0.5 = 5.0` | descend at 5.0 m/s |
| `0.0` | `3.0 + 4 × 0.0 = 3.0` | descend at 3.0 m/s |
| `-0.5` | `3.0 - 4 × 0.5 = 1.0` | descend at 1.0 m/s |
| `-0.9` | `3.0 - 4 × 0.9 = -0.6` | climb at 0.6 m/s |

The vertical-speed request is converted into an equivalent altitude target for
the existing altitude cascade:

```text
altitude target = measured altitude - descent request / altitude position gain
```

This is how `dy` influences thrust: it does not directly set throttle. It moves
the altitude target, and the altitude controller decides the required thrust.

### 5. Altitude target becomes collective thrust

<p align="center">
  <img src="altitude-thrust-loop.svg" width="100%" alt="Altitude target cascade through position and velocity control, gravity and tilt compensation to throttle PWM.">
</p>

The position controller converts altitude error to target vertical velocity.
The velocity controller converts vertical-speed error to acceleration. Thrust is
then `mass × (gravity + acceleration)`, bounded by available motor thrust.

#### Calculating the altitude position gain

The altitude position controller is proportional. It converts altitude error in
meters into a vertical-speed target in meters per second:

```text
altitude error = target altitude - measured altitude
vertical-speed target = position gain * altitude error
```

The configured position gain is `1.5 /s`. Its units are `(m/s)/m`, which reduce
to inverse seconds. Before phase limits are applied, the current gain produces:

| Altitude error | Calculation | Vertical-speed target |
|---:|---:|---:|
| `+10 m` | `1.5 × 10` | `+15 m/s`, then phase-limited |
| `+2 m` | `1.5 × 2` | `+3 m/s` climb |
| `+0.5 m` | `1.5 × 0.5` | `+0.75 m/s` climb |
| `-1 m` | `1.5 × -1` | `-1.5 m/s` descent |

Select a starting gain by choosing the vertical speed desired at a representative
altitude error:

```text
position gain = desired vertical speed / representative altitude error
```

For example, requesting 3 m/s at 2 m error gives `3 / 2 = 1.5 /s`. A simple
first-order interpretation is `time constant approximately 1 / gain`; a gain of
`1.5 /s` corresponds to approximately `0.67 s` before speed limits, the velocity
loop and vehicle dynamics are considered.

Approach guidance starts with a desired descent speed rather than a fixed
altitude. It therefore converts that speed into an equivalent altitude target:

```text
altitude target = measured altitude - descent request / position gain
```

For a 3 m/s descent with a `1.5 /s` gain, the altitude target is 2 m below the
measurement. The position controller then reconstructs the requested speed:

```text
vertical-speed target = 1.5 * (-2) = -3 m/s
```

A higher position gain reacts faster but is more sensitive to barometer noise
and MAVLink delay. A lower gain is smoother but takes longer to remove altitude
error. Tune it together with the vertical-speed PI loop rather than in isolation.

Because a tilted vehicle directs part of its thrust horizontally, collective
thrust is divided by `cos(roll) × cos(pitch)`. The divisor is bounded so extreme
tilt cannot request unbounded thrust.

### 6. Attitude targets become body-rate commands

The attitude controller compares the requested roll, pitch and heading with the
MAVLink quaternion. Quaternion attitude error is multiplied by a gain of 4/s and
limited to body rates of `(2, 2, 1)` rad/s.

Search and reacquisition override only yaw rate. Roll and pitch still use the
attitude controller, allowing the vehicle to remain level while rotating.

### 7. Body rates and thrust become RC channels

The requested body rates are inverted through Betaflight's configured ACTUAL
rate curve. Collective thrust is mapped through the motor model to throttle PWM.
The resulting channel order is:

| Channel | Value | Meaning |
|---:|---|---|
| 1 | 1000–2000 PWM | roll-rate stick |
| 2 | 1000–2000 PWM | pitch-rate stick |
| 3 | 1000–2000 PWM | throttle stick |
| 4 | 1000–2000 PWM | yaw-rate stick |
| 5 | 1000 or 2000 PWM | arm switch |
| 6 | 1000 PWM | Acro mode selection |

Neutral roll, pitch and yaw are 1500 PWM. Disarmed throttle and arm are 1000
PWM. All channel values are validated before transmission.

### 8. MSP delivery and Betaflight inner loop

The 240 Hz mission loop stores the newest RC tuple in `RcStream`. The stream
sends the latest tuple at 50 Hz as an MSP `SET_RAW_RC` request over TCP port
5761. Intermediate 240 Hz updates are intentionally replaced, not queued.

Betaflight interprets the RC values using its configured Acro rate curve. Its
internal gyro PID closes the fast body-rate loop, and its mixer converts
collective thrust plus roll, pitch and yaw corrections into four normalized
motor outputs. Python does not calculate individual motor commands.

### 9. Motor outputs advance physics and close the loop

At every 240 Hz physics step, the backend sends simulated FDM sensor data to
Betaflight on UDP port 9003 and receives four normalized motor outputs on UDP
port 9002. The outputs are validated, reordered to the project rotor layout and
multiplied by maximum motor speed.

PyBullet applies motor dynamics, calculates propeller forces, applies rotor
thrust and body effects, and advances one simulation step. The new simulated
sensor values return to Betaflight; Betaflight publishes the next MAVLink
measurements; and the loop repeats.

The three important update rates are therefore:

| Loop | Rate | Responsibility |
|---|---:|---|
| Physics and outer mission | 240 Hz | simulation, guidance and target generation |
| MSP RC stream | 50 Hz | deliver newest roll, pitch, throttle and yaw request |
| MAVLink attitude and altitude | 20 Hz | measured outer-loop feedback |

The outer controller must tolerate the slower MAVLink feedback. If measurements
become stale, the mission fails instead of falling back to simulator truth.

## Individual control loops

### Camera steering

```mermaid
flowchart LR
    Image[Camera frame] --> Error[Red centroid dx and dy]
    Error --> Steering[Pitch visibility heuristic and roll correction]
    Steering --> Attitude[Roll and pitch targets]
```

### Altitude position

```mermaid
flowchart LR
    Target[Phase altitude target] --> Error[Target minus measured altitude]
    Baro[MAVLink VFR_HUD altitude] --> Error
    Error --> P[Altitude P gain 1.5]
    P --> Limit[Phase vertical speed limit]
```

### Vertical thrust

```mermaid
flowchart LR
    Target[Vertical speed target] --> PI[Vertical PI gains 2 and 1]
    Baro[Filtered altitude derivative] --> PI
    PI --> Accel[Acceleration limited to 5 m per second squared]
    Accel --> Thrust[Gravity and tilt compensation]
    Thrust --> PWM[Throttle PWM]
```

### Attitude

```mermaid
flowchart LR
    Target[Roll pitch heading targets] --> Error[Quaternion error]
    Measured[MAVLink ATTITUDE orientation] --> Error
    Error --> P[P gain 4 per second]
    P --> Rates[Limited body rates]
    Rates --> RC[ACTUAL curve to MSP RC]
```

### Firmware stabilization

```mermaid
flowchart LR
    RC[MSP rates and throttle] --> BF[Betaflight rate controller and mixer]
    Gyro[Simulated gyro] --> BF
    BF --> Motor[Motor outputs]
    Motor --> Physics[Physics and sensors]
    Physics --> Gyro
```

### Concurrent data flow

```mermaid
sequenceDiagram
    participant Main as Flight loop
    participant Vision as Camera worker
    participant Sensors as MAVLink worker
    participant BF as Betaflight
    Vision-->>Main: Latest dx and dy from result queue
    BF-->>Sensors: ATTITUDE and VFR_HUD
    Sensors-->>Main: Latest fresh sensor sample
    Main->>BF: MSP rates and throttle
    BF-->>Main: Motor outputs
```

## Parameters and logs

| Setting | Default |
|---|---|
| Launch position | (-20, -18, 0.015) m, body/world Y points left |
| Takeoff | 20 m altitude target, 4 m/s climb cap |
| Approach | descent request = altitude-scaled 3.0–3.5 + 4 × dy m/s, limited to ±8 m/s |
| `dy` pitch scaling | 50% at 20 m and below, rising to 100% at 40 m |
| Landing | 1.5 m/s descent cap |
| Search/reacquire yaw | 0.35 / 0.25 rad/s |
| Takeoff settling | within 1 m, vertical speed within 1.5 m/s, for 0.3 s |
| Initial search deadline | 30 simulation seconds |
| Mission landing deadline | 150 simulation seconds |
| Body rate limits | (2, 2, 1) rad/s |
| Camera | 640 by 480, 90 degrees horizontal FOV, 30 FPS per simulation second |
| Detector | R >= 120, G and B <= 105, R >= 1.35G, minimum 8 pixels |
| Vision freshness | 120 physics steps |
| Physics / MSP RC | 240 Hz / 50 Hz |
| MAVLink attitude / altitude | 20 Hz / 20 Hz |

Colored STATE logs show transitions. VISION logs at 10 Hz report image error,
MAVLink altitude, estimated climb, commanded pitch/thrust and RC. TIMING logs at
1 Hz report simulation speed, FDM rate, capture rate, SHM and detector FPS.

Earlier contact results used ground-truth velocity control and do not validate
this sensor-only camera controller.

## Validation

The MAVLink-only sensor controller achieved renderer-confirmed truck contact
from the 20 m takeoff at 17.167 s and from the 40 m takeoff at 22.225 s. All 17
focused tests passed. Different launch points, wind, telemetry delays and image
noise remain unverified. Pitch adaptation is a visibility heuristic, not a
measured maximum-speed optimizer.
