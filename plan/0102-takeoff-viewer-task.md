# Takeoff image viewer task option

Goal: expose the rendered takeoff and SHM camera viewer in the existing VS Code example picker.
Steps: add one labeled option targeting examples.takeoff_image_viewer.main.
Interface changes: one task picker option; reuse existing render extras and NVIDIA environment.
Verification: parse tasks JSON and verify option entry point exists.
Scope/defaults: preserve default selection and other tasks.
Status: Implemented and verified; uncommitted
Implementation commit: Pending

Outcome: tasks JSON parsed and selected module path verified.
