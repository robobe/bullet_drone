# Example-owned MSP TCP client

Goal: remove bullet_drone dependency on msp-link while preserving Betaflight examples.

Implementation: extract TCP-only MSP framing/parser, transport, client, RC stream and command helpers into examples/common/msp; migrate SITL adapters, tests and docs; update uv metadata and lock.

Interfaces: preserve Client, FlightCommands, MspError and TcpTransport behavior, timeouts, reconnection, RC scheduling and cleanup. Exclude SerialTransport. Preserve optional sitl extra as an empty compatibility extra.

Scope: examples only; no separate wheel, core physics changes or sibling project changes. Retain validated implementation rather than rewrite the protocol.

Verification: migrated protocol tests, affected and full repository suites, import check without msp_link, available SITL checks.

Status: Implemented and verified; uncommitted
Implementation commit: Pending

Verification: focused MSP/altitude-hold suite 27 passed, 2 skipped; isolated imports succeeded without msp_link; real headless altitude-hold flight completed at 37.217 simulation seconds, landed/disarmed and closed MSP plus owned firmware. Full suite rerun: 582 passed, 3 skipped; uv lock --check and git diff --check passed. Live Acro square and terrain flights were not rerun.
