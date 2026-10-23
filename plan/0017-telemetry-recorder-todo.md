# Centralized telemetry recorder TODO

## Goal

Document the next simulation telemetry milestone in `docs/todo.md`.

## Implementation steps

1. Create the TODO document with an unchecked centralized recorder milestone.
2. Record shared queue ownership, background output, overflow handling and verification expectations.
3. Identify telemetry schemas and YAML configuration as prerequisites.

## Interface changes

None. This milestone only documents future work; it does not implement schemas, configuration or a recorder.

## Verification

Checked the document against the agreed scope; `git diff --check` passed. Runtime tests were not run because only Markdown documentation changed.

## Scope and defaults

One bounded queue per recorder, non-blocking immutable frame submission, centralized lifecycle and drop reporting, and background batched file or network output. Simulation telemetry only.

## Status

Complete; committed.

## Implementation commit

`0e629400438b6f1d2fcbb4e37b40853ed3108aa7`
