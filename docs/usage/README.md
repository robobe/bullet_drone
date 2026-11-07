# Using the simulation

This guide uses the `altitude_hold` example to show how to run the simulation
and read its state and sensor values after every physics step.

## 1. Run altitude hold

From the repository root, run:

```bash
uv run python -m examples.altitude_hold.main --headless
```

The example creates a simulation, resets it, then repeatedly requests an RC
command from the altitude-hold mission and advances one physics step. The
headless option disables the PyBullet window and wall-clock pacing.

## 2. Understand the observation

`RcSimulation.reset()` and `RcSimulation.step()` return a `DroneObservation`.
The altitude-hold demo passes the reset observation and each new step
observation to its optional `on_observation` callback. That lets an application
inspect or record values without changing the physics loop.

An observation contains:

- `ground_truth`: simulated drone state, including episode ID, step index,
  simulation time, position, orientation, linear and angular velocity, and
  rotor speeds.
- `sensors.imu`: body-frame specific force and angular velocity, when enabled.
- `sensors.barometer`: pressure and measured altitude, when enabled.
- `sensors.rangefinder`: distance and validity, when enabled. The altitude-hold
  example enables it, so a terrain hit appears as a distance in centimetres.
- `attitude_estimate`: an optional orientation estimate derived from the IMU.
  The altitude-hold example selects this source and prints the estimate.

Sensor slots can be `None` when a sensor is disabled or unavailable. The state
and sensor readings are immutable values. Sensor readings include the episode,
step, and simulation-time stamps for the measurement.

## 3. Print each observation to standard output

Run this from the repository root. It uses altitude hold's existing callback
and prints once for the initial reset observation and once after every physics
step:

```bash
uv run python - <<'PY'
from examples.altitude_hold.demo import run_demo


def print_observation(observation):
    state = observation.ground_truth
    imu = observation.sensors.imu
    barometer = observation.sensors.barometer
    print(
        f"episode={state.episode_id} step={state.step_index} "
        f"time={state.simulation_time_s:.3f}s "
        f"position_m={state.position_m} "
        f"velocity_m_s={state.linear_velocity_m_s} "
        f"imu_specific_force_m_s2="
        f"{None if imu is None else imu.specific_force_m_s2} "
        f"imu_angular_velocity_rad_s="
        f"{None if imu is None else imu.angular_velocity_rad_s} "
        f"barometer_pressure_pa="
        f"{None if barometer is None else barometer.pressure_pa} "
        f"barometer_altitude_m="
        f"{None if barometer is None else barometer.altitude_m}",
        flush=True,
    )


run_demo(gui=False, on_observation=print_observation)
PY
```

`print()` writes observation data to standard output. The example's Loguru
messages are diagnostics and go to standard error, so they remain separate from
the values printed by the callback. Redirect or pipe standard output if you
want to capture the readings, for example:

```bash
# After saving the snippet as print_altitude_hold.py:
uv run python print_altitude_hold.py > observations.txt
```

For sensor units, coordinate frames, configuration, and measurement details,
see [IMU and barometer sensors](../design/sensors.md). For the full observation
type definition, see `DroneObservation` in `src/bullet_drone/models.py`.
