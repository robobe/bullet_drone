# Camera viewer FPS display

Goal: show actual displayed frame rate in the separate SHM camera window.
Steps: count displayed frames over monotonic one-second intervals; show FPS status label; test timing, idle and frame accounting; document.
Interfaces: viewer-only FPS counter; no protocol changes.
Verification: focused viewer tests and desktop Tk integration check.
Scope/defaults: displayed FPS, not renderer or physics rate; count at most one frame per GUI refresh; show zero when idle.
Status: Implemented and verified; uncommitted
Implementation commit: Pending

Outcome: 8 viewer tests passed; desktop Tk idle FPS label/update and close verified; git diff --check passed. Full rendered flight not repeated.
