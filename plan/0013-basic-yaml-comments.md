# 0013: Basic YAML configuration comments

## Goal

Apply the YAML documentation rule to every existing field in configs/basic.yaml.

## Implementation steps

Add adjacent comments describing each field and section, units and frames, valid values, defaults and omission behavior where applicable. Preserve all configuration values.

## Interface changes

None.

## Verification

Passed: parsed YAML equals the previous version, YamlConfigLoader accepts the annotated file, and git diff --check is clean. No runtime tests needed for comments only.

## Scope and defaults

Comments only in the existing basic configuration; do not add settings or alter values.

## Status

Complete; committed.

## Implementation commit

`5d9375636df529290aaecc92f450062fcc18b044`
