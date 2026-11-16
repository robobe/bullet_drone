# Complete simulation feedback

`flight/drone_1/feedback` uses PUB/SUB on the existing telemetry endpoint, default TCP 5555. The external-render publisher offers every observation to a bounded latest-value slot before camera scheduling. Its worker converts observations to protocol dataclasses, and ZmqIo serializes and sends dictionaries. Slow consumers can miss intermediate samples; network waits never enter the physics loop. Repeated stopped observations may carry identical simulation timestamps.

The contract is defined in `bullet_simulation_protocol.feedback`. Frozen dataclasses support strict `from_dict()` parsing and `to_dict()` export, including tuple-to-array conversion. The old feedback JSON Schema is removed. This is a breaking payload replacement with no legacy format support.

| Field | Contents |
| --- | --- |
| `header` | World frame and simulation time |
| `drone_id` | Configured drone identity |
| `ground_truth` | Header, position_m, orientation_xyzw, altitude_m, linear_velocity_m_s, angular_velocity_rad_s, rotor_speeds_rad_s |
| `sensors.imu` | Header, specific_force_m_s2, angular_velocity_rad_s |
| `sensors.barometer` | Header, pressure_pa, altitude_m |
| `sensors.rangefinder` | Header, distance_cm, valid |
| `sensors.gps` | Header, latitude_deg, longitude_deg, altitude_m, velocity_enu_m_s |
| `attitude_estimate` | Header and estimated orientation_xyzw |

Every header has `frame_id` and `time_stamp: {sec, nanosec}`. Time is simulation elapsed time, normalized to integer seconds and nanoseconds, restarting on reset. Nested readings retain their measurement times. The world frame is Bullet's Z-up world, quaternions are XYZW body-to-world rotations, and ground-truth angular and linear velocities are world-frame quantities. Ground-truth altitude is exactly position_m[2]. Body measurements use `<drone_id>/base_link`: IMU specific force and gyro rates are body coordinates, and rangefinder reports distance along its body-down mounted ray. Barometer altitude uses world Z. GPS uses `earth`, with geographic latitude/longitude, MSL altitude and east/north/up velocity. Estimated attitude remains separate from ground truth.

Disabled sensors and an unavailable attitude estimate are null. A rangefinder without a return remains a reading with `distance_cm: null, valid: false`. IMU acceleration is specific force, not an additional ground-truth acceleration estimate. No synthetic sensors or measurement values are added.

The payload has no schema_version, source, episode ID, step index, armed, mode, last_applied_sequence, owner_id, state, reason or takeoff_height_m. Consumers can infer a reset when simulation time decreases, but cannot uniquely identify episodes from timestamps. Existing lifecycle, camera headers, image sample source identity and render pose contracts remain separate.

Tests and builds have not been run by user request. Manual checks should cover all sensors enabled and disabled, quaternion and frame interpretation, altitude equality, time reset, and a slow subscriber while physics continues.
