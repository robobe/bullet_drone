# Runnable SITL binaries under bin

Goal: separate runnable standard and MAVLink firmware from source/build trees.
Steps: install atomic executable copies after successful setup; migrate CLI defaults and docs; copy existing builds; ignore generated ELF files.
Interfaces: defaults become bin/betaflight_SITL.elf and bin/betaflight_mavlink_SITL.elf; source-directory arguments and --sitl-binary overrides unchanged.
Verification: isolated setup fixtures for both variants, install permissions and failure preservation, CLI tests, headless altitude flight and MAVLink startup.
Scope/defaults: copies, no symlinks; retain source/compiler outputs under .sitl; historical plans unchanged.
Status: Implemented and verified; uncommitted
Implementation commit: Pending

Outcome: 51 affected tests passed, 3 skipped; both setup variants tested in isolated roots for success, dirty-source refusal and build failure preserving prior binary. All four CLI defaults/overrides verified. Real headless standard-binary altitude flight completed at 37.400 simulation seconds with cleanup; MAVLink binary MSP startup and heartbeat verified with cleanup. Shell syntax and diff checks passed. Existing builds copied, not recompiled; full suite not rerun.
