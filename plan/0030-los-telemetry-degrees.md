# Focused LOS telemetry in degrees

Status: implemented, verified and committed. Implementation commit: `610c92bb54fcfc39df37af0b532515e2b2354da6`.

## Goal and scope
bullet_app publishes tracking/front/los on existing port5558 and MCAP records it.
Include only bearing(deg), LOS rate(deg/s), TTC(seconds), confidence(validity,
qualification reason, dimensionless relative residual), timestamps and state.
Keep LOS/control internals and existing radian mission diagnostics unchanged.
No new port/thread/dependency. Commits were subsequently authorized by the user;
pushing remains outside scope.

## Implementation
- Shared version1 LosTelemetry contract/schema/topic; strict finite/null validation.
- App mapper consumes public NavigationResult/Diagnostics and converts rad*180/pi.
- Mission-thread/shared-socket publication at30Hz plus immediate mission/navigation
  state, stage, reason/activity changes. Publish inactive markers before LOS and on
  stop/fault/completion/reset. Clear invalid/inactive estimates; retain last state
  for ending episode and clear it on new episode. Preserve supplied TTC residual
  for unqualified active fits. No calibrated confidence probability is invented.
- Active generation time comes from NavigationResult; inactive transition time is
  app monotonic time. Preserve physics source stamp/simulation time separately;
  telemetry and navigation sequences distinguish publication from control results.
- Add commented recorder profile topic, documentation and focused tests.

## Verification
Completed with dependencies and library/controller behavior unchanged:
- Affected suites:227 passed (bullet_msgs104, bullet_app36, mcap_bagger68,
  bullet_render19). Includes conversion pi->180, negative signs/rates, qualified
  and insufficient/not-closing/noisy/stale TTC, confidence residuals, strict
  null/finite/validity/schema checks,30Hz sampling from100Hz ticks and immediate
  state/lifecycle changes, source identity/reset, nonblocking retry/shutdown.
- Exact-topic real TCP + MCAP readback verifies degree values and final inactive
  publication; suffix topics are ignored. All Python package builds passed.
- Real-display flight/LOS integration passed. MCAP retained44 LOS messages and33
  active bearing samples matched degree conversions of the existing radian mission
  diagnostics. Existing828 exact bbox/pose associations and app-loss watchdog,
  restart and reset checks also passed.
- Real-display collision integration passed across2 episodes, with deliberate
  first-report loss/retry. Both bags contained final inactive COMPLETED LOS markers
  within the existing250ms recorder drain. Confirmed end events remained deduped.

Reports: `/tmp/bullet-los-flight/report.json`,
`/tmp/bullet-los-collision/report.json`. Build log:
`/tmp/bullet-los-python-build.log`. Integration bags were read back then removed;
normal recordings still use bullet_render/recordings. Committed on user request;
no pushes performed.

No native source changed, so native rebuild/frame-only checks and unchanged
standalone drone/LOS suites were not repeated for this logging-only change.
The real integrations exercised production physics, renderer, app and recorder;
collision integration retains the previously documented deterministic trajectory.

## Implementation commits

- bullet_msgs: `09cfc485bb8cf9884744e3afc4e5ac32dbe49f28`
- bullet_app: `7c6613f1288b780b6b8b199a5ef58ea54c3a4d1f`
- bullet_drone: `610c92bb54fcfc39df37af0b532515e2b2354da6`
- bullet_render: `6e2c1b6c932ac4429633513b30ef31222e83c567`
- los_tracking and mcap_bagger: unchanged; no new commits needed.
