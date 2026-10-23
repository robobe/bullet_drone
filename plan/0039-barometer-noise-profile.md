# 0039: Realistic barometer noise profile

Goal: enable a representative real-sensor pressure noise level in configs/basic.yaml.

Steps: set sensors.barometer.pressure_stddev_pa to 2.3 Pa, retaining the existing
pressure-to-altitude model, zero fixed bias and seeded sampling. Document the
source and approximate 0.19 m altitude standard deviation near sea level.

Reference: Bosch BMP388 datasheet, table 8, high resolution with IIR off:
https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp388-ds001.pdf

Interfaces/defaults: supplied YAML profile only; omitted library setting remains
zero. Existing source selection and IMU settings are untouched. This uses a
representative RMS noise amplitude, not a complete device timing/filter model.

Verification: load YAML, check sampled altitude noise and reset repeatability,
and run existing sensor/configuration tests. No flight or hardware check.

Status: Implemented. All 95 sensor, configuration and attitude checks passed.
A 3000-sample check at 10 m measured 2.289 Pa pressure and 0.191 m altitude
standard deviation, with repeatable reset. The sensor-default test now clears
demo pose selections before disabling sensors so its fixture remains independent.
`git diff --check` passed.
Implementation commit: `ce54722cebbcea1ccac78eb70008de4b5e23c14f`.
