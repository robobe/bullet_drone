# Wind GUI, physics and renderer flow

Goal: explain local wind application and renderer publication in the wind example README.
Steps: add left-to-right Mermaid blocks with local calls, applied readback, wind-state and pose topics over default TCP 5555; exclude images and external-RC controls.
Interfaces: none.
Verification: render exact Mermaid with CLI and check diff whitespace.
Scope/defaults: documentation only; endpoint tcp://127.0.0.1:5555 is configurable.
Status: Implemented and verified; uncommitted
Implementation commit: Pending

Outcome: exact Mermaid block rendered successfully to /tmp/wind-flow.svg; git diff --check passed. Runtime tests not rerun for documentation-only change.
