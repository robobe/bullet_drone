# Camera pose and image header

The simulation publishes a validated frozen `PoseMessage` dataclass as a dictionary on `render/drone_1/pose`, default PUB/SUB TCP 5555. It contains exactly header, drone_id, position_m and orientation_xyzw. Python contracts live in bullet_simulation_protocol.pose and are re-exported by bullet_msgs.pose. Pose JSON Schemas and schema_version are removed.

```json
{"header":{"fid":240,"time_stamp":{"sec":1,"nanosec":0}},"drone_id":"drone_1","position_m":[0,0,3],"orientation_xyzw":[0,0,0,1]}
```

The header uses the current simulation step_index as fid and elapsed simulation time as sec/nanosec. Position is Bullet world XYZ metres, Z up; orientation is the XYZW body-to-world unit quaternion. The configured pose-source selection remains, including optional barometer altitude and estimated attitude. The configured drone ID is preserved. Godot binds its single drone object to the first valid drone ID and rejects different IDs until restart.

Godot applies this transform to the drone and its camera, then captures an image. The default fps=30 schedules 30 poses per simulation second; a slow simulation produces fewer per wall second. Stopped refresh stays 1 Hz. Queues retain only the latest value and physics never waits for network delivery.

Version-3 image descriptors copy header and drone_id unchanged from the captured pose. Simulation time is separate from the existing monotonic capture_ns, which describes renderer arrival time. SHM binary layout/session/slot/sequence are unchanged. Readers still support descriptor versions 1 and 2. Frame matching uses an internal renderer token, not a new wire-pose field. Image readiness requires the owned renderer SHM session, expected dimensions, drone ID and initial pose header.

fid/time_stamp can repeat while stopped and restart on reset. They identify a simulation sample within a run but are not globally unique. Image session and image sequence distinguish individual deliveries. There is no legacy source object, velocity, episode/stream identity, lifecycle or sensor bundle in pose. Full simulation telemetry stays separate. Pose-source collision reporting and the obsolete pose-schema recorder selection have been removed.

Tests and builds were not run. Rebuild the renderer before checking images and header propagation manually.
