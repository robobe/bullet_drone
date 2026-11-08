# Local protocol libraries

These independent uv workspace packages ship as separate wheels. Install both through `bullet-drone[render]`.

- `bullet-simulation-protocol`: import `bullet_simulation_protocol`. Owns simulation/render configuration, pose, lifecycle, flight, spawn, wind and scenario contracts, JSON schemas and transport-profile validation.
- `bullet-image-transport`: import `bullet_image_transport.shm`. Owns image descriptors, the POSIX SHM binary format, readers/writers and slot helpers.

The renderer consumes the unchanged JSON/SHM protocol and does not import Python. Existing SHM magic and slot prefixes are retained for compatibility. Sibling projects still use bullet_msgs independently.

Run checks with `uv run --extra render pytest libs/bullet_simulation_protocol/tests libs/bullet_image_transport/tests`.
Build with `uv build --package bullet-simulation-protocol` and `uv build --package bullet-image-transport`.

Outgoing and incoming contracts retain existing dictionaries and validators to avoid changing call semantics during extraction. Unrelated perception, LOS and recording contracts are excluded.
