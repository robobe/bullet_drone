# Global GPS coordinates in flight logs

## Goal
Show latitude, longitude and MSL altitude alongside local ENU in GPS example logs.

## Steps and interfaces
Preserve decoded GLOBAL_POSITION_INT coordinates as a geographic_position field
on example-owned Feedback. Log explicitly labelled degree/metre values. Keep
local feedback and navigation unchanged. Update the example documentation.

## Verification
Extend the existing decoding test and run focused example tests.

## Scope and defaults
Use original MAVLink measurements, not a reverse conversion of rounded ENU.
No core interfaces, dependencies or physics changes.

## Status
Implemented and verified: focused tests 14 passed, 1 opt-in flight skipped;
whitespace check passed. No full flight rerun for a display-only addition.

Implementation commit: Pending
