# Separate simulation protocol and image transport libraries

Goal: remove bullet_msgs from bullet_drone while preserving wire compatibility.

Steps: create two independently packaged libraries under libs, migrate required contracts and validation, update callers and workspace dependencies, verify tests and wheels.

Interfaces: bullet_simulation_protocol owns transport profiles and simulation contracts; bullet_image_transport owns descriptors and SHM. Existing topics, versions, validation, binary headers and slot names remain unchanged. No unrelated app/recording/LOS contracts are migrated.

Verification: existing protocol and repository tests, wheel resource inspection and dependency checks.

Scope/defaults: sibling projects remain unchanged except parent workspace membership and lock metadata. Existing dict interfaces are retained to avoid unnecessary conversion and protocol changes.

Status: Implemented and verified; uncommitted
Implementation commit: Pending

Verification outcome: repository suite 574 passed, 3 skipped; extracted contracts 111 passed; renderer/publisher regression rerun 25 passed. Both wheels built and inspected (13 simulation schemas, one image schema). No dataclass conversion was introduced: existing validated dictionary contracts were preserved. Image transport explicitly depends on simulation source validation.
