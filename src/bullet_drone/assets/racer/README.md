# Racer drone assets

Adapted from [gym-pybullet-drones](https://github.com/learnsyslab/gym-pybullet-drones/tree/01563120e060c9233a61f9cf00e6d4e5a8b331ab/gym_pybullet_drones/assets), commit `01563120e060c9233a61f9cf00e6d4e5a8b331ab`.

The upstream `racer.urdf` references `cf2.dae`, a scaled Crazyflie mesh. The mesh's original author metadata credits Lakshman Kumar. These are the upstream racer's visuals, not a newly modeled racing frame.

Local modifications:

- Split `cf2.dae` into a body mesh and four propeller meshes, preserving the scene transforms, 3x scale, and upstream visual yaw of 55 radians.
- Add individual propeller visuals to the existing fixed propeller links. Front means body positive X: `prop0_link` and `prop3_link` are orange, while the rear pair are blue.
- Preserve the original collision cylinder (radius 0.06 m, height 0.025 m), inertial definition, fixed links, and custom rotor properties.

At runtime the YAML mass overrides the nominal 0.830 kg URDF mass. The base inertia is scaled by the same mass ratio. Rotor links remain massless. Ideal rotor thrust and reaction torque use the upstream coefficients, converted from RPM squared to rad/s squared. Forces act on the base at the rotor inertial-origin positions. Capacity is derived from nominal mass and thrust-to-weight ratio and does not increase when YAML mass is increased.

Meshes and URDF are bundled with the Python package; simulation does not download assets or require Gymnasium. Reproduce the assets with `uv run python tools/build_racer_assets.py` from the repository. The upstream MIT license is included in [LICENSE.upstream](LICENSE.upstream).
