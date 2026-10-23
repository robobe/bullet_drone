# 0041: Noise-tolerant hover transition

Goal: allow the app to confirm hover with the current noisy, filtered barometer
without changing flight control or widening motion limits.

Implementation: keep a bounded 241-reading, one-second altitude history in the
app mission, populated from fresh barometer feedback and simulation timestamps.
Retain the boundary reading that covers the window. Require three readings and
one second of history, then fit a centered linear altitude/time trend. Clear on
episode/source reset, missing barometer data or gaps beyond the existing sensor
freshness limit. Use trend velocity for hover and yaw-settle decisions only.

Keep +/-0.25 m altitude, +/-0.2 m/s trend speed, +/-5 degree attitude/heading,
three-second hover and existing target centering/yaw-settle conditions. Keep
controller estimator input, LOS handoff, freshness and timeout behavior intact.
Expose settling_vertical_speed_m_s and settling_window_s in hold diagnostics.
Update app documentation. No configuration or wire-schema additions.

Verification: regression with current physics YAML, real sensor/filter/physics
and synthetic camera; noisy dwell success; sustained motion and excursion
rejection; irregular/duplicate/missing samples; resets; unchanged controller
velocity input; existing search, image qualification and LOS handoff checks.

Limits: trend detects average motion with up to about one second of lag. It is
not instantaneous stability detection. No hardware or full renderer flight test.

Status: Implemented. The new noisy-profile regression first reproduced
FAULT/takeoff_timeout before the change. All 83 app checks now pass, including
real noisy/filtered physics through search and LOS_APPROACH, constant climb and
descent rejection, excursions, duplicate/missing/irregular data, identity resets,
and unchanged controller velocity input. Diff whitespace checks passed.

Cause: instantaneous noise-amplified velocity repeatedly restarted the three-
second dwell. The mission now fits sensor altitude history solely for transition
qualification, with regression coverage for noise and real sustained motion.
No temporary instrumentation was added to repository source.
Implementation commit: `9b85374f70a0e87ad7f8d873922847e540258d66`.
