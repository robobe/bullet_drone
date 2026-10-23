# 0038: Startup configuration banner

Goal: print all resolved external-RC startup configuration to stdout.

Steps: serialize the effective simulation, stabilization, vehicle, render,
transport and advertised platform settings, with source paths, in a readable
JSON banner before initializing physics. Include defaults and the headless
override. Keep diagnostic logging on stderr and library constructors quiet.

Interfaces: external-RC startup stdout gains one labeled JSON snapshot. No CLI,
configuration or wire-protocol change. Other example entry points are unchanged.

Verification: assert captured stdout matches the loaded YAML settings, defaults,
overrides and other loaded configuration. Run external-RC, spawn and publisher
regressions. No flight or hardware validation is needed for this output change.

Status: Implemented. All 17 external-RC, spawn and publisher checks passed;
`git diff --check` passed. Captured stdout covers YAML and built-in defaults,
resolved overrides and a single banner across resets/spawn replacement.
Implementation commit: `55562afcc05b73de4b76ab2cbe165de15b276806`.
