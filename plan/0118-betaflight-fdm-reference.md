# Betaflight FDM reference

## Goal
Create simulation/sitl_fd.md explaining upstream FDM, sensor mapping and communication ports.

## Steps
Review pinned upstream sitl.c, target.h, sitl_gyro.h and serial_tcp.c; document packet layout, updateState, bridge modes and ports; verify offsets and links.

## Interfaces
Documentation only. No public API changes.

## Scope and defaults
Upstream only; use requested path; preserve docs/simulation/sitl_fdm.md. Plain-text diagram. No firmware changes or live tests.

## Verification
Passed: packet size and contiguous offsets checked, five pinned source links fetched successfully, port and conversion source reviewed. No live firmware tests.

## Status
Implemented and verified; uncommitted.

## Implementation commit
Pending.
