# Godot installation for external-render builds

Status: Implemented and verified; uncommitted. Implementation commit: Pending.

## Goal

Document the Godot editor and export-template installation required to build the
external renderer, while making clear that running an exported package does not
require a separate Godot installation.

## Implementation steps

1. Link to the official Godot 4.7.2 Linux download.
2. Document installing the editor executable on `PATH` and its matching export
   templates.
3. Add version and template verification commands before the existing build
   command.

## Interfaces and scope

Documentation only. The build script, generated renderer, dependencies, and
runtime behavior do not change. No commit or push requested.

## Verification

- Confirmed the documented editor check reports
  `4.7.2.stable.official.ed1daf0bf`.
- Confirmed the matching Linux templates exist at the documented directory.
- Confirmed the official archive and export-template documentation links describe
  the pinned download and template-manager workflow.
- `git diff --check` passed.
- The renderer was not rebuilt because the implementation is documentation only.
