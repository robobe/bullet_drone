# 0042: ICM-42688-P IMU noise model

Goal: add datasheet-based white-noise density, sensor low-pass filtering and
configurable residual bias random walk to the physics-owned IMU.

Interfaces: ImuSettings gains noise_model (per_sample default or density),
accelerometer_noise_density_m_s2_sqrt_hz, gyro_noise_density_rad_s_sqrt_hz,
filter_enabled (false default), accelerometer_cutoff_hz (30), gyro_cutoff_hz (60),
accelerometer_bias_random_walk_m_s2_sqrt_s and gyro_bias_random_walk_rad_s_sqrt_s.
Vectors default to zero. Existing fixed biases, per-sample deviations and seed
remain. Reject density mode combined with nonzero per-sample noise.
ImuSensor gains optional keyword sample_period_s, required for density/filtering;
RcSimulation supplies its timestep. Reject enabled cutoffs at/above Nyquist.

Model: manufacturer densities are one-sided amplitude spectral densities. Draw
per-axis white noise with sigma = density / sqrt(2*dt). Advance residual bias by
coefficient*sqrt(dt)*Gaussian(0,1). Add true body-frame signal, fixed/residual bias
and white noise, then optionally apply independent accel/gyro first-order filters.
For s=sin(pi*cutoff*dt), alpha=2*s/(sqrt(1+s*s)+s) gives the selected digital -3 dB
cutoff. Initialize filters from the first reading. Use nominal dt on reset, actual
measurement intervals thereafter. Clear histories/bias/RNG on episode reset.
Preserve legacy sequences when new options are omitted. No output/schema changes.

Demo: density mode, accel densities [0.00063743225, 0.00063743225, 0.0006864655]
m/s^2/sqrt(Hz), gyro [0.00004886921906]*3 rad/s/sqrt(Hz), filtering enabled at
30/60 Hz. Fixed bias and drift remain zero as a calibrated baseline. Validate and
document settings and model assumptions, without dependencies or a firmware layer.

Sources: TDK ICM-42688-P datasheet tables 1-2, DS-000347 revision 1.6:
https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000347-icm-42688-p-v1.6.pdf
Hardware references: SpeedyBee F405 V4 and Holybro Pixhawk 6C specifications.

Verification: legacy compatibility; density units, white-noise statistics and
axis independence; filtered variance/correlation at multiple rates; random-walk
elapsed-time scaling and repeatability; reset/validation/Nyquist/lifecycle;
unchanged commanded physics; attitude, telemetry, renderer and noisy-hover-to-LOS
regressions. Record measured outcomes before completion.

Limits: effective 240 Hz simulation feed, not full kHz firmware or motor vibration.
No temperature drift, quantization, saturation or hardware calibration/replay.

Status: Implemented. All 337 focused physics-side checks and all 83 app checks
passed (420 total), including the noisy sensor/filter hover-to-LOS regression.
New tests verify exact legacy sequences, SI conversions, white and filtered
variance/correlation at 240/480/960 Hz, digital half-power cutoff, nominal/actual
intervals, elapsed-time drift variance, repeatable resets, invalid settings and
Nyquist, and unchanged commanded physics with the demo's density profile.
Diff whitespace validation passed. No hardware or full renderer flight test.
Implementation commit: `5842d499d976f2d0945c185e5d9a485b63e2accd`.
