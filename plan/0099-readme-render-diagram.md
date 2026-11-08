# README simulation and renderer diagram

Goal: insert the requested simulation/render block diagram at README line 27.
Implementation: add a left-to-right Mermaid flowchart showing simulation observations passing through the ExternalRenderer wrapper and RenderPublisher to Godot. Exclude image reception and SHM reading from this diagram.
Interface changes: none.
Verification: render the exact Mermaid block with Mermaid CLI and check the diff.
Scope/defaults: documentation only; preserve existing example menu.
Status: Implemented and verified; uncommitted
Implementation commit: Pending

Verification outcome: Mermaid CLI rendered the exact added block to SVG successfully; git diff --check passed.
