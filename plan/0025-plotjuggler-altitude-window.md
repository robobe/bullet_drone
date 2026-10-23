# PlotJuggler altitude opening window

## Goal and scope

Set the altitude opening view in `plotjuggler/layouts/xy_alt_zmq.pj4.xml`
to Y = 0–5 m and a 360-second time span, with a 360-second stream buffer.
Keep the existing absolute-time anchor, XY panel, curve bindings and MCAP loader.
The layout supplies an opening viewport, not a persistent live axis lock:
PlotJuggler 4 auto-fits live samples and does not restore custom fixed limits
from this layout format. No application patch or global preference changes.

## Implementation steps

1. Keep altitude bounds at 0 and 5; make decimal and integer-nanosecond X
   bounds agree on a 360-second interval from the existing saved left edge.
2. Increase this layout's streaming buffer from 5 to 360 seconds.
3. Document the opening view and live auto-fit limitation.

## Interfaces

Only existing PlotJuggler XML range and streaming-buffer attributes change.

## Verification

Passed: XML parses; decimal and integer-nanosecond bounds agree on exactly
360 seconds; altitude bounds are 0–5 m; the stream buffer is 360 seconds; XY
and MCAP loader elements are unchanged, including the loader CDATA.
GUI reload/live range locking was not verified; no live lock is claimed.

## Status

Complete; committed. Implementation commit: `20652a1a2f3bfbdc321f3d29340e8ed1612e7bd9`.
