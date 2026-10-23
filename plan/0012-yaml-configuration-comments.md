# 0012: YAML configuration comments

## Goal

Require explanatory comments for every field in YAML configuration files.

## Implementation steps

Add an AGENTS.md rule requiring adjacent comments explaining each configuration field's meaning and usage, including units, accepted values and defaults where applicable. Keep comments current when fields change.

## Interface changes

None; repository guidance only.

## Verification

Inspected the added rule; `git diff --check` passed. No runtime tests needed for this guidance-only change.

## Scope and defaults

Add the rule only; annotating existing YAML files is outside this change.

## Status

Complete; committed.

## Implementation commit

`5d9375636df529290aaecc92f450062fcc18b044`
