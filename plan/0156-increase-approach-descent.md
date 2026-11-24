# Increase visual approach descent

## Goal

Increase the baseline descent during image-guided approach while preserving the
existing image-error response and vertical-speed safety limit.

## Implementation steps

1. Raise the low-altitude baseline from 3.0 to 4.0 m/s and the full-scale
   baseline from 3.5 to 4.5 m/s.
2. Update the example guide's equations, examples, and approach summary.
3. Update focused descent tests and run them with lint.

## Interface changes

No public interface changes. Existing controller constants retain their names.

## Verification

Run focused image-guidance tests and Ruff on changed Python files.

## Scope and defaults

Keep the 4.0 m/s per-unit `dy` gain, altitude interpolation, and 8.0 m/s
approach vertical-speed cap unchanged.

## Status

Implemented; focused 20 m / 40 m descent regression passed, Ruff passed, and
`git diff --check` passed. Rendered-flight verification was not run.
Implementation commit: Pending.
