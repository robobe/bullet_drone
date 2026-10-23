# Control diagram SVG

## Goal
Replace the PNG with an editable SVG describing the implemented RC input and
internal Python control chain.

## Implementation steps
- Draw software controller/altitude PID → RC encoding → 18-channel RcInput →
  RcSimulation.step() → RC decoding.
- Split collective thrust from angle/rate commands. Show Angle roll/pitch through
  attitude P, Acro bypassing it, and yaw rate entering rate PID in both modes.
- Combine thrust and torque in the mixer; show bounded motor rad/s, ideal motors,
  propeller thrust/reaction torque, and PyBullet stepping.
- Show ground-truth feedback to altitude, attitude and body-rate control.
- Use editable SVG shapes/text: blue control blocks, orange physics blocks and
  dashed green feedback. Link from the README and remove the obsolete PNG.

## Interface changes
None; documentation asset replacement only.

## Verification
Parse SVG XML, render and visually inspect labels, arrows, clipping and readability.
Check documentation references after deleting the PNG.

## Scope/defaults
Show current implementation only. No Betaflight, joystick hardware, motor lag,
simulated IMU or camera/ADRC branches. No behavior or dependency changes.
Earlier plan files are absent in this workspace; retain the agreed number 0003.

## Status
Implemented and verified; uncommitted.

Verification outcome: SVG XML parsed successfully; marker references and README
link validated. Rendered in headless Chrome and visually inspected twice,
correcting connectors that crossed labels. All labels and feedback arrows are
readable without clipping. Obsolete PNG removed; no PNG references remain.
Application tests were not rerun because only documentation assets changed.

Implementation commit hash: Pending (no commit requested; checkout has no `.git`).
