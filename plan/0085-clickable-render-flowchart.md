# Clickable render flowchart

## Goal

Let readers open the source methods from a flowchart of the altitude-hold observation-to-render path.

## Implementation steps

1. Add a Mermaid flowchart with clickable source links next to the sequence diagram.
2. Verify that Mermaid renders the diagram and emits links for its method nodes.
3. Document that link behavior depends on the Markdown viewer's Mermaid security settings.
4. Add this plan to the plan menu.

## Interface changes

None. Documentation-only navigation links target existing source files and lines.

## Verification

- Render the flowchart with Mermaid CLI configured to allow links and inspect generated SVG anchors.
- `git diff --check`.

## Scope and defaults

- Link to the relevant implementation methods using direct GitHub source URLs on the current default branch.
- Keep the existing sequence diagram unchanged.

## Status

Implemented and verified; uncommitted.

- Mermaid CLI rendered the flowchart to SVG with `securityLevel: loose`.
- Confirmed the SVG contains links for all six method nodes.
- `git diff --check` passed.

## Implementation commit

Pending
