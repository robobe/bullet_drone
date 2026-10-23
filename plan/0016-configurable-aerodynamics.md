# 0016: Configurable aerodynamic force chain

## Goal

Implement default-enabled single-drone aerodynamic effects with independent modules and Python/YAML controls, while preserving caller-owned loops and solver-owned gravity/contact.

## Implementation steps

1. Add validated force and wind settings. Every new effect has its own enable flag, default true. Document all YAML fields, units and default/omission behavior.
2. Add body drag, rotor drag, angular damping, rotor inflow, blade flapping, gyroscopic torque, ground effect and local propwash modules under forces. Keep calculations independent of PyBullet.
3. Add constant ambient wind or seeded exponentially correlated Gaussian gusts driven by simulation time. Defaults: enabled, calm constant wind, zero gust standard deviation, correlation time 1 s, seed 2; reset restarts gust state/sequence.
4. Preserve speed-only baseline PropellerModel. Compose baseline outputs with a contextual force chain using pre-step state, actual speeds, airflow and read-only surface queries. Return four rotor forces, body force and aggregate body torque. Validate all results before application.
5. Apply rotor forces once at offsets, body force at center of mass and body torque once. Use physical spin signs opposite reaction signs. Add read-only downward queries excluding the drone/links for ground effect.
6. Add independent effect/configuration/physics tests and explicit ideal configurations for existing analytical checks. Run full suite and all three headless missions with new defaults; preserve routes, settling, landing/disarm and time limits, tuning gains only if necessary.
7. Update force table, chain SVG, design menu and domain glossary.

## Models and defaults

Air density 1.225 kg/m³; rotor radius 0.0635 m. Body quadratic drag CdA [0.02,0.02,0.03] m². Rotor linear drag coefficients [1e-5,1e-5,1.1e-5] in rad/s-compatible units. Angular damping [0.002,0.002,0.001] N*m/(rad/s).

Inflow multiplier clamp(1-horizontal_speed/tip_speed,0.5,1), configurable coefficient 1. Flapping coefficient 0.1. Gyro rotor inertia 1e-5 kg*m², body torque -Omega cross H. Ground multiplier 1+0.1*(radius/(2*distance))² times squared positive normal alignment on the gain, capped 1.2; query range 1 m, minimum distance .01 m.

Propwash uses baseline thrust to avoid circular coupling: momentum-theory induced speed, expanding wake with spread .1, inverse-area decay and Gaussian radial decay; combined speed bounded to 20 m/s. Body airflow sample offset [0,0,-.03] m approximates downwash exposure in the coplanar asset. Exclude each rotor's own wake from its inflow. Rotor-point velocity includes body rotation.

## Interface changes

SimulationConfig gains force/wind settings. New project-owned surface query contract, immutable hit and final force-output data. Baseline propeller/motor interfaces, sensor observations, MCAP and example CLIs remain unchanged.

## Verification

Completed independent formula/direction/scaling/bounds/frame/disable checks; gyro signs, scene hits/self-exclusion, wake direction/decay/superposition, wind cancellation/gust statistics/reset, invalid-output/lifecycle checks and ideal baseline recovery.

- Final full suite: `uv run --locked pytest -q` — 384 passed, 1 skipped (optional MCAP dependency unavailable).
- Aerodynamic regression tests: 40 passed; combined aerodynamic/shared-geometry checks: 62 passed.
- All-default headless altitude hold completed at 32.825 simulated seconds.
- All-default headless Angle square completed at 54.371 simulated seconds.
- All-default headless Acro square completed at 54.383 simulated seconds.
- Existing 5 cm flight altitude accuracy, routes, settling, touchdown/disarm and time limits retained.
- Read-only scene rays exclude reserved drone collision-group bit 14, including all links; raised/flat surface and no-hit checks pass.
- Both control/force SVGs parsed and visually inspected; all 15 numbered tips retained. Every basic YAML key has a preceding explanatory comment. git diff --check passed.
- Actual GUI viewer not exercised. Coefficients remain teaching approximations; no calibrated-vehicle claim.

## Agreed implementation adjustment

Ground effect revealed insufficient downward altitude integral authority: the original example gains hovered near 2.7 cm and could not meet the existing 2 cm touchdown threshold. Independent tuning raised altitude velocity PID Kp to 6, Ki to 4 and integral contribution limit to 2 m/s², retaining output limit 3 m/s². This restores landing and square altitude accuracy without relaxing acceptance criteria. This gain adjustment was authorized in the agreed plan.

## Scope and defaults

Teaching approximations, not calibrated vehicle data. New effects enabled in existing demos, calm wind by default. YAML access remains library-only. No motor lag, battery model, multi-drone world, estimator, new dependency or shared runner.

## Status

Complete; committed.

## Implementation commit

`7619e7dea916908c387b36ef22f404dafa600406`
