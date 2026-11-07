# Terrain following: MAVLink monitor and MSP Acro control

Build the separate firmware with `bash tools/setup_betaflight_mavlink_sitl.sh`.
Run `uv run --locked --extra mavlink --extra sitl --extra render python -m examples.terrain_following_mavlink.main`.
Add `--headless` to omit the viewer.

The mission follows the same 3 m clearance route at up to 3 m/s. MSP sends Acro body-rate sticks, throttle and arm commands on TCP 5761. MAVLink telemetry is received on TCP 5762 and printed once per second. The monitor waits up to 10 seconds for its first heartbeat and marks readings stale after 3 seconds. Monitoring loss does not disable MSP flight safeguards. GPS is disabled; no simulated position or clearance is relabeled as firmware telemetry. The worker closes before SITL teardown.

The original example and firmware binary remain separate. Only one SITL example can run at a time.
