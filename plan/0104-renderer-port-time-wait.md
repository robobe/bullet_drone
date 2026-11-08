# Renderer startup after TCP TIME_WAIT

Goal: allow immediate renderer restart after its old TCP connections close.
Steps: reproduce real TIME_WAIT on image/control ports; add regression for both adapters; use SO_REUSEADDR on preflight sockets to match ZeroMQ binding semantics; retain occupied-port rejection.
Interfaces: no changes.
Verification: real socket regression, existing renderer/Acro/wind tests, direct startup and cleanup.
Scope/defaults: fix only the preflight probe; never kill other listeners or silently change endpoints.
Status: Implemented and verified; uncommitted
Implementation commit: Pending

Outcome: real TIME_WAIT regression failed in all four adapter/endpoint combinations before fix; 35 tests passed, 1 skipped after fix including occupied-port rejection. Two consecutive NVIDIA/Godot starts each delivered a matching camera frame and cleaned up successfully. Full mission/suite not rerun.
