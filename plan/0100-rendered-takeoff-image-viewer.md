# Rendered takeoff with separate image viewer

Goal: one example starts a rendered takeoff/hold/landing and a separate SHM image window.
Steps: reuse existing rendered mission; add standalone Tk viewer subscribing to image descriptors and reading FrameReader; own child lifetime and failures; document and test.
Interfaces: new examples.takeoff_image_viewer.main entry point and examples.takeoff_image_viewer.viewer standalone entry point. Reuse renderer/render-config CLI options. No core interfaces or dependencies change.
Verification: RGB/BGR/stride conversion, SHM roundtrip, child cleanup/failure tests, actual desktop rendered flight when available.
Scope/defaults: existing 3 m takeoff, 10 s hold, landing; no image recording; Tkinter system support required. Viewer exits with mission, and closing it early stops the mission with cleanup.
Status: Implemented and verified; uncommitted
Implementation commit: Pending

Verification outcome: 22 focused tests passed; both CLI help paths passed; real NVIDIA/Godot desktop mission completed takeoff, hold, landing and disarm at 32.825 simulation seconds with a separate 642x482 Tk camera window. Publisher closed with zero drops; process exited successfully. Full repository suite not rerun. Desktop screenshot capture unavailable (X get_image error); window presence/size and successful mission checked instead.
