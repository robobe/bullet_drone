# Bullet image transport

`bullet-image-transport` provides camera image descriptors, POSIX shared-memory
slots and frame readers/writers. Import its helpers from
`bullet_image_transport.shm`. Pixels stay in local shared memory; ZeroMQ carries
only the notification describing where to read them.

## Topics and messages

| Topic / resource | Message / purpose | Direction | Default TCP port | Contract |
| --- | --- | --- | --- | --- |
| `camera/front/image` | Image descriptor: dimensions, encoding, sample identity and SHM slot reference | Godot renderer → Python readers/viewers | 5556 | [image.json](src/bullet_image_transport/schemas/image.json) |
| `/bullet_msgs_<session>_s<slot>` | Binary header followed by image pixel bytes | Renderer/writer → local SHM readers | None, local SHM | [shm.py](src/bullet_image_transport/shm.py), `HEADER` and slot helpers |

The renderer binds `tcp://127.0.0.1:5556` by default. Readers connect and subscribe
using `topic_prefix('camera/front/image')`. The endpoint is configurable through
`image_endpoint` in the transport profile. The library supports exact topics;
`camera/front/image` is the current renderer's camera topic.

## Descriptor fields

| Field | Meaning / units |
| --- | --- |
| `version` | Descriptor version 1 or 2; version 2 requires `source`. |
| `session` | Publisher session identity, 32 lowercase hexadecimal characters. |
| `slot` | Shared-memory slot index, 0–2. |
| `shm_name` | Slot name matching the session and index. |
| `width`, `height` | Positive image dimensions in pixels. |
| `encoding` | `rgb8` or `bgr8`, three 8-bit channels per pixel. |
| `step` | Bytes per row, including padding; at least `width * 3`. |
| `bytes` | Pixel buffer size: `step * height`, at most 64 MiB. |
| `sequence` | Positive frame sequence used to reject duplicate or overwritten frames. |
| `capture_ns` | Capture timestamp in nanoseconds. For version 2, equals `source.measurement_us * 1000`. |
| `source` | Version-2 sample identity: physics/episode/stream IDs, step, simulation time and monotonic measurement time. |

The image notification is **one ZeroMQ frame**, unlike the simulation protocol's
two-frame topic/JSON messages. Its layout is:

```text
UTF-8 topic + NUL + uint32 little-endian JSON length + UTF-8 JSON descriptor
```

Descriptor JSON is limited to 1024 bytes. The wire field is named `version`, not
`schema_version`. Version 1 has no source stamp; version 2 validates source identity
using the simulation protocol library.

## Shared-memory layout and ownership

Three slots hold a 128-byte little-endian header (`<8s6I4Q64s`) followed by pixels.
The header contains magic `BMSHM01\0`, binary version 1, slot index, width, height,
encoding code, row stride, pixel byte count, sequence, capture timestamp,
completion marker and session identity. Descriptor version 2 keeps this binary
header unchanged. The historical `bullet_msgs_` slot prefix is retained for native
renderer compatibility; it does not require the Python bullet_msgs package.

`FrameReader.read(notification)` returns `(metadata, owned_pixel_bytes)` or `None`
for normal skips such as duplicate, busy, missing or overwritten slots. Malformed
metadata and invalid slot identity/layout raise errors. Readers check file type,
owner, permissions and size, and use nonblocking advisory locks while copying.

Call `FrameReader.close()` to release mappings and file descriptors. Readers do
not unlink slots. `FrameWriter.close()` removes its own slots; the renderer adapter
also cleans up slots owned by its Godot child when that child exits.

## Imports and schema access

```python
from bullet_image_transport import schema_bytes
from bullet_image_transport.shm import FrameReader, topic_prefix

reader = FrameReader('camera/front/image')
try:
    # Subscribe with topic_prefix(...), receive one notification, then:
    # frame = reader.read(notification)
    pass
finally:
    reader.close()
```

The [separate camera-viewer example](../../examples/takeoff_image_viewer/README.md)
shows a complete subscriber and image window. Simulation pose, wind and lifecycle
messages belong to [bullet-simulation-protocol](../bullet_simulation_protocol/README.md).
