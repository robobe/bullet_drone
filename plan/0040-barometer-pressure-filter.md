# 0040: Configurable barometer pressure filter

Goal: smooth noisy barometer pressure before altitude conversion for render/app use.

Implementation: add filter_enabled (strict boolean, default false) and
filter_time_constant_s (positive finite seconds, default 0.1) to BarometerSettings.
Apply an exponential low-pass filter after noise/bias using measurement intervals.
Initialize from the first measured pressure and clear history on reset/spawn.
Validate pressure before/after filtering and commit history only on successful
conversion. Keep the existing pressure_pa and altitude_m fields consistent.

Enable filtering with a 0.1 s time constant in configs/basic.yaml, keeping its
2.3 Pa noise. Library omission defaults preserve unfiltered behavior. Implement
inside BarometerSensor with standard-library math, without a new abstraction.
Update sensor documentation and output-field descriptions; no schema shape change.

Verification: disabled compatibility, regular/irregular step response, pressure
inversion, seeded noise reduction, reset/chronology, invalid settings/readings,
and existing config, telemetry, source-selection and external-RC regressions.

Limits: approximately 0.1 s lag during steady climb, no removal of fixed bias,
drift or outliers. Raw pressure telemetry and full flight validation are excluded.

Status: Implemented. All 290 focused sensor, configuration, external-RC,
attitude, telemetry, publisher, spawn and RC-simulation checks passed. A 3000-sample
stationary check at 10 m measured 0.191 m raw and 0.030 m filtered altitude
standard deviation after settling. `git diff --check` passed. Full flight and
hardware validation were not run.
Implementation commit: `ce54722cebbcea1ccac78eb70008de4b5e23c14f`.
