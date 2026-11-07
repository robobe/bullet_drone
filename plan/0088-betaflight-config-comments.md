# Explain Betaflight SITL profile settings

Status: Implemented and verified. Implementation commit: `aa3e3079d81f8331412b4f6063ae51b0314bc9ca`.

## Goal

Add plain-language comments to `configs/betaflight.config` so readers can
understand the purpose of each Betaflight CLI setting used by the examples.

## Implementation steps

1. Explain the quad mixer, RC channel map, receiver features, disabled sensor
   fusion sources, AUX mode ranges, and Angle limit.
2. Explain yaw PID overrides, ACTUAL rates and rate values, throttle handling,
   and the final `save` command.
3. Preserve all CLI commands and values, then run the focused Acro example
   configuration test.

## Interfaces and scope

Comments only. No Betaflight setting, firmware version, CLI syntax, or example
behavior changes.

## Verification

- `uv run --locked pytest -q tests/test_position_square_acro_example.py` — 11 passed, 1 skipped.
- Compared active CLI lines before and after; every command and value is unchanged.
- `git diff --check` — passed.

## Implementation commit

`aa3e3079d81f8331412b4f6063ae51b0314bc9ca`
