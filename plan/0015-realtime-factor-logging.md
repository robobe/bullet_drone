# 0015: Real-time-factor logging

## Goal

Show how elapsed simulation time compares with wall-clock time in all three examples.

## Implementation steps

- Add a small shared example timing helper using time.monotonic and Loguru, without configuring sinks or owning an execution loop.
- Start measurement after simulation, GUI and mission initialization; share the existing pacing clock origin.
- Report cumulative simulation elapsed seconds, wall elapsed seconds and RTF = simulation elapsed / wall elapsed once per wall second and on successful completion.
- Measure after pacing sleeps so control, physics, sensors, rendering overhead and intentional waiting are included. Exclude setup/cleanup.
- Integrate altitude hold, Angle square and Acro square without changing pacing or physics. Document meaning and GUI/headless expectations in their READMEs.
- Keep reporting interval/format precision as named module-top constants and guard zero elapsed time.

## Interface changes

Example-only timing helper; no library, CLI, YAML, control or telemetry interfaces change.

## Verification

Controlled-clock tests for factors 1.0/2.0/0.5, reporting cadence, final summaries and zero elapsed time. Existing example tests and all three headless missions must preserve flight results. Controlled-clock integration checks confirm sleep is included and setup/cleanup are excluded.

Verification completed:

- Timing helper tests: 11 passed.
- Timing plus existing tests for all three examples: 56 passed.
- Headless altitude hold: completed at 23.225 simulation seconds; final log sim=23.225s wall=0.905s RTF=25.675x.
- Headless Angle square: completed at 44.496 simulation seconds; final log sim=44.496s wall=2.013s RTF=22.109x.
- Headless Acro square: completed at 44.500 simulation seconds; final log sim=44.500s wall=2.124s RTF=20.955x.
- Mission times and final physics results match prior runs. Wall times/factors depend on machine load; concurrent verification is not a benchmark.
- git diff --check passed. GUI pacing checked with a controlled clock; actual GUI viewer and unrelated tests not rerun.

## Scope and defaults

Always-on diagnostics in all three examples, GUI and headless. Cumulative ratio since loop startup; one-second wall-clock reporting interval plus final summary. Measures execution speed, not physical accuracy. No new dependencies.

## Status

Complete; committed.

## Implementation commit

`eb83baa1dd7bc05b6d49a34b528815adcdb1f9c7`
