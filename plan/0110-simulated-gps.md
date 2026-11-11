# Embedded GPS sensor in simulation

Goal: simulate GPS inside the drone sensor pipeline, deliver its geographic position
through Betaflight FDM `position_xyz`, and retain local metre coordinates for physics
and rendering.

Status: Implemented and verified; uncommitted
Implementation commit: Pending

## Current behavior and constraints

- `DroneState.position_m` and renderer poses contain local world XYZ metres.
  `drone.initial_position_m` already specifies the local spawn position.
- Sensors are separate from ground truth, constructed in `RcSimulation`, sampled
  by `DroneEnvironment`, and returned in `SensorReadings`.
- `betaflight/fdm.py:pack_fdm()` currently sends `[0, 0, pressure-derived altitude]`
  in FDM position, truth velocity in velocity, and measured pressure in field 17.
- Checked local Betaflight source at e0b7bb01b17b21351057e9ead2d1ab39dd44fa16:
  position means `[longitude degrees, latitude degrees, altitude metres]`.
  Gazebo mode mirrors horizontal coordinates around the first received position;
  it also ignores the pressure field and derives barometer pressure from position Z.
  Both shipped firmware profiles currently disable GPS.
- Consequently, filling position alone cannot provide independent GPS altitude
  and barometer noise. A small, explicit firmware compatibility change is needed
  for a complete GPS sensor. Do not silently replace the barometer with GPS altitude.

## Scope and defaults

- Add ideal GPS first: immediate valid fix, no noise, drift, acquisition delay,
  dropout, antenna offset, or configurable receiver rate. Sample each physics step;
  document the virtual receiver's actual firmware update cadence and fixed fix data.
- GPS is opt-in (`sensors.gps.enabled: false` by default), preserving existing
  profiles and direct environment users. No new dependency or wire schema.
- Define local world X as east, Y as north, Z as up for geographic conversion.
  This does not rotate physics or change existing controller/body conventions.
- Configure the geographic coordinate of local world `[0, 0, 0]`, not of the
  reset/spawn point. Keep `drone.initial_position_m` as the independent local spawn.
  Thus changing spawn or resetting never moves the geographic world origin.
- Use one documented altitude datum, mean-sea-level metres, matching the virtual
  GPS consumer. No geoid conversion or terrain georeferencing in this milestone.

## Implementation steps and interface changes

1. **Configuration and geographic mapping.** Add `GpsSettings` under
   `SensorsSettings` and YAML `sensors.gps` with `enabled`, `origin_latitude_deg`,
   `origin_longitude_deg`, and `origin_altitude_m`. Require all three origin fields
   when enabled; do not invent a real location. Validate booleans, finite values,
   geographic ranges, and supported near-pole limits. Use a small, documented local
   tangent approximation with WGS84 curvature radii at the origin: north displacement
   changes latitude, east changes longitude, altitude is origin altitude plus world Z.
   Wrap longitude at the dateline. Document the local-flight accuracy limit and
   reject unsupported polar origins. Add adjacent YAML comments and an opt-in example.

2. **Sensor and lifecycle.** Add immutable `GpsReading` with observation identity,
   latitude/longitude degrees, altitude metres, and measured ENU velocity m/s.
   Add optional `gps` at the end of `SensorReadings` to preserve positional callers.
   Implement `GpsSensor.reset/sample` through the existing sensor contract, export
   it from `sensors`, and wire optional injection into `DroneEnvironment` and
   construction into `RcSimulation`. Keep sensor results out of `DroneState`.
   Reset generates the correct coordinate for the current local spawn, with a new
   episode identity and the same configured geographic origin.

3. **Betaflight compatibility.** Keep the current Gazebo IMU mapping and 18-double
   FDM layout. Add a narrowly scoped, reproducible build option/patch for this
   Python bridge: consume field-17 measured pressure directly, and accept geographic
   coordinates without the Gazebo horizontal reflection. Do not switch the whole
   bridge to legacy mode, since that also changes IMU conventions. Preserve the
   default unmodified firmware build for existing GPS-disabled use. Integrate the
   optional build into the existing setup process without discarding dirty sources,
   and identify the compatible binary so a GPS profile cannot silently launch an
   incompatible binary. Inspect both standard and MAVLink build paths.

4. **FDM and profile wiring.** When GPS is enabled, encode position as
   `[reading.longitude_deg, reading.latitude_deg, reading.altitude_m]`, and velocity
   as `[east, north, up]` from that reading. Validate identity/timestamps, finite
   values and geographic ranges just as for existing sensors. Continue sending
   the independent barometer pressure in field 17. Preserve existing encoding and
   errors when GPS is disabled. Add a matching opt-in firmware profile using the
   virtual GPS provider and GPS feature, with explicit configuration/binary pairing.
   Keep GPS rescue and magnetometer changes outside scope. Define disabled-GPS
   behavior explicitly: retain the existing GPS-disabled firmware profile, rather
   than feeding zero coordinates as an apparent valid fix.

5. **Rendering and consumers.** Keep the existing local `position_m` path and
   existing altitude/attitude source choices. Do not send degrees to the renderer
   or subtract the initial spawn from its existing positions. Rendering needs no
   geographic round trip. Audit sensor/config exports, positional constructors,
   example configuration loaders, recording/telemetry consumers, packaged physics
   configuration, setup fixtures and example session factories for compatibility.
   Add raw GPS telemetry only through an existing suitable sensor publication path;
   a new telemetry protocol or GPS-driven render mode is outside scope.

6. **Documentation.** Update sensor design, FDM field table and firmware setup
   instructions, design menu, and relevant example README. Explain world origin
   versus spawn, axes, units, altitude datum, local approximation, and the firmware
   compatibility requirement. Update this plan and its menu entry with outcomes.

## Verification and acceptance

- Config tests: enabled/disabled/omitted GPS, required origin, invalid coordinates,
  nonfinite values, strict booleans, unknown keys, old profiles still load.
- Sensor tests: zero and nonzero spawn; east/north/up movement; matching velocity;
  reset and episode stamps; dateline wrapping; polar rejection; known small-offset
  reference values independent of the implementation formula.
- FDM tests: unchanged 144-byte layout; longitude/latitude order and ENU signs;
  stale/missing enabled GPS rejection; original GPS-disabled behavior preserved.
- Firmware setup tests: optional patch application, dirty-source preservation,
  compatible binary/profile selection and failure without replacing a good binary.
- Live SITL: verify GPS fix and geographic coordinates using MSP raw GPS, then move
  east and north and check reported direction/course. Check nonzero origin altitude
  and spawn height. Change barometer pressure with fixed GPS altitude, then GPS
  altitude with fixed pressure, proving independence. Restart/reset with a changed
  spawn and verify there is no first-packet origin reflection.
- Rendering regression: the same local trajectory produces the same pose positions
  with GPS on/off, including nonzero spawn, without any protocol changes.
- Run affected config, sensor, environment/simulation, bridge and setup tests;
  perform the existing headless Betaflight altitude smoke test with GPS enabled.

## Risks and deferred work

The shared upstream position-Z consumer makes full GPS a firmware compatibility
change, reversing the earlier preference for an unmodified SITL binary only for
this opt-in path. If maintaining that small patch is unacceptable, restrict a
separate milestone to horizontal GPS with explicitly coupled altitude; do not
claim independent GPS/barometer simulation. GPS loss, realistic receiver timing,
noise, long-distance geodesy and navigation/rescue are deferred.

## Implementation outcome

- Added optional GPS settings, immutable readings, WGS84 local mapping and reset/step
  wiring. Geographic origins are required on enable; the approximation rejects
  horizontal distances beyond 1 km and origins beyond +/-85 degrees latitude.
- GPS FDM explicitly validates sensor identity, coordinates, finite data and the
  firmware's altitude/velocity integer conversion limits. GPS-disabled packets
  retain their existing layout and pressure-derived Z.
- Added opt-in standard/MAVLink profiles and `--gps` setup variants, atomic
  installation, compatibility markers, managed-session pairing checks, and source
  patch reversal on exit. Unmanaged external firmware owners must validate their
  binary/profile with `validate_gps_firmware`; UDP has no capability negotiation.
- Added `configs/gps.yaml` and `--gps-origin LAT LON ALT` to the existing headless
  altitude example. Updated the explicit Iris profile with disabled GPS defaults.
- Retained renderer protocols and local positions. Audited the external-RC sensor
  feedback and render publisher: no geographic topic exists, so raw readings stay
  in observations and firmware GPS telemetry rather than creating a new schema.
- No implementation commit requested or created. Implementation commit remains Pending.

## Verification outcome

- Full Python suite: 627 passed, 5 skipped. Final affected config/sensor/simulation/
  FDM/setup/render/Iris tests: 175 passed, 2 inapplicable marker cases skipped.
- Both real GPS firmware variants built successfully from the pinned local source
  checkouts; both checkouts were clean after build cleanup. Default installed
  firmware binaries were retained.
- Live standard and MAVLink tests passed GPS fix/12 satellites, nonzero origin and
  spawn, east/north coordinates and course, independent pressure and GPS altitude,
  and restart with a different spawn (2 tests, 47.30 s). The expanded live standard
  test passed again; the final MAVLink test passed in 30.33 s, including arming
  readiness, near-zero VFR_HUD altitude after arm at a nonzero MSL location, and
  a 6–11 m relative increase after a 100 Pa pressure reduction. The first armed
  test attempt hit calibration blocking; it now waits for normal firmware readiness.
- Existing GPS-enabled headless 3 m takeoff/10 s hold/land/disarm mission completed
  at 37.283 simulation seconds, final local Z 0.0135 m, with owned resources closed.
- Shell syntax and diff whitespace checks passed. No GUI flight or hardware tests.

