# Course force inventory and implementation chain

## Goal
Review the linked course module's force table against this repository and produce
a source-linked Markdown table and an editable SVG with matching numbered tips.

## Steps and interfaces
- Preserve the course's effect indices 1–13. Append contact (14), from its
  unnumbered inventory, and battery/KV (15), from its non-force inputs.
- Record classification, physical location, current status, proposed implementation
  module and independent verification for every item. Explain control/mixer role
  separately from actual physical-force calculation/application.
- Cite the course and local code, clearly distinguishing course status from this
  project's implementation and recommendations.
- Create force-implementation-chain.svg with numbered callouts, native hover
  tooltips and links to document rows, marking future models explicitly.
- Link the artifacts from the design menu and README; retain current control SVG.

## Verification
Validate row/tooltip ID agreement, links and SVG XML, render and visually inspect.
Review force ownership, frames/units and double-counting hazards against local code.

## Scope/defaults
Documentation only; no physical models, APIs, dependencies or runtime changes.
Gravity/contact belong to PyBullet; thrust at rotor positions already generates
lever torque. Wind is airflow input, motor dynamics and battery behavior are not
standalone forces. Mass/inertia/collision properties documented separately.

## Status
Complete; implemented, verified and committed.

Verification outcome:
- Reviewed the course Module 6 table and course physics source against local
  controller, mixer, motor, propeller, force, physics and configuration code.
- All 15 document anchors match the 15 SVG badge indices and links. SVG parses
  as XML; every badge has a native title tooltip and keyboard-focusable link.
- All relative document links resolve; README/design-menu links are present.
- SVG rendered in headless Chrome and visually inspected, correcting overflow,
  connector/text crossings and state/frame labels. Current and future effects
  are visibly differentiated; default control SVG remains available.
- Document distinguishes current tests from recommended future checks, and
  course implementation status from this repository. Gyroscopic spin/reaction
  conventions and missing read-only aero context are explicitly recorded.
- Runtime tests were not rerun: no simulation code or dependencies changed.

Implementation commit hash: `ac453654b7378854b8e449280df88016a2ab556b`.
