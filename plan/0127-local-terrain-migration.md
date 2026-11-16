# Local terrain migration

## Goal and steps
Make custom/terrain authoritative, preserve manifest, update all active physics/render/export consumers and documentation, verify tests and renderer build, then remove only verified old terrain files and empty directory.

## Interfaces and defaults
Paths change only; preserve terrain bytes and packaged runtime paths. Include sibling renderer and workspace profile consumers. No commits or dependency upgrades.

## Verification
Passed: 25 focused terrain/profile/renderer tests; 6 terrain/profile tests rerun after removal; Godot exporter check-only; full renderer package build; duplicate byte comparisons including preserved manifest; active consumer search; whitespace checks. No live flight test. Old workspace terrain directory removed.

## Status
Implemented and verified; uncommitted.

## Implementation commit
Pending.
