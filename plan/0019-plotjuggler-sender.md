# Ground-truth streaming to PlotJuggler

## Goal

Stream the Angle square example through a sender child process outside src, using a bounded queue and UDP/JSON, with a reusable XY and Z/time layout.

## Implementation steps

1. Add tools/telemetry_sender with strict, commented YAML configuration, spawn-process lifecycle, non-blocking submission, latest-sample wall-clock rate limiting and UDP output.
2. Add --telemetry-config to the Angle square CLI and an optional observation callback after reset/each successful step. Load settings/start the sender before physics and clean up on completion, failure and interruption.
3. Preserve the full existing ground-truth schema inside a transport envelope {timestamp, topic, sample}; use simulation time for the top-level timestamp.
4. Add plotjuggler/layouts/ground_truth_position.xml with UDP/JSON input, embedded timestamps, equal-scale XY and Z over simulation time. Add separate VS Code tasks for PlotJuggler and the telemetry-enabled example.
5. Document use, counters, shutdown, episode clearing and measured overhead; update the design menu and recorder roadmap.

## Interfaces and defaults

run_demo gains an optional observation callback; ordinary runs are unchanged. Sender code/config stay outside src. Defaults: IPv4 host 127.0.0.1, port 9870, send_rate_hz 5.0, queue_capacity 256, startup_timeout_s/shutdown_timeout_s 3.0. YAML fields have adjacent usage/default comments; reject unknown keys and invalid types/ranges/nonfinite values.

The example launcher owns one multiprocessing.Queue and starts one sender using spawn. Immutable DroneState objects cross the queue; JSON and socket work stay in the child. Sender retains the newest available state and sends at most once per wall-clock interval, without repeats/catch-up bursts; normal shutdown sends the final available state within its deadline. Track overflow drops separately from rate-limit skips, sends and errors. Runtime sender failure disables streaming/reports once while flight continues. Invalid configuration remains an actionable startup error.

## Installed PlotJuggler compatibility

Verified PlotJuggler4 4.0.0 with UDP Server 0.9.1 and JSON Parser 1.1.0. This build restores plot definitions/source selection but not live connection/parser configuration, and requires an explicit start action. UDP address/port and embedded timestamp are configured in the native dialog and remembered as app preferences. Equal XY scale is also an app preference (default enabled). Document these steps rather than invent unsupported XML/CLI switches.

The build rejects startup layouts with an empty catalog. Include one unplotted layout_bootstrap.csv fixture referenced by relative path; pending plot intents bind only to udp/data fields when the live stream arrives. The fixture supplies no synthetic position samples. Normal shutdown retries the final state if its loop submission was rejected by the full queue.

## Verification

Test settings, schema-conformant received datagrams, timestamps, latest selection/rate limits, overflow, sender failures and bounded lifecycle. Verify localhost UDP and live plots/source restoration in installed PlotJuggler. Compare capture latency/real-time factor with streaming off/on. Run full suite and git diff --check.

Outcome: 569 tests passed; git diff --check passed. Native PlotJuggler startup layout restoration exited successfully, and both live panels were visually verified over a complete paced GUI flight. The 54.371 s flight reported RTF 1.000, 13,050 accepted states, 272 sends, 12,778 rate skips, and zero overflow/unsent/errors. A real process-group Ctrl+C check returned exit 130 and confirmed the sender was reaped. Paired headless measurements found median parent submission ~10 us, p95 15–18 us, and a maximum 3.53 ms; accelerated runs overflowed the bounded queue without blocking/failing the flight. Headless RTF measurements were noisy and imply no speedup/portable overhead guarantee; detailed measurements are in the sender README. Final rejected-state retry and bounded low-rate shutdown have focused checks. No runtime dependencies or src changes were needed.

## Scope

Ground truth live preview only; two panels. Native 3D, file recording and other topics are deferred. UDP has no delivery acknowledgement. Accelerated headless runs intentionally yield fewer samples under a wall-clock cap. No new runtime dependencies or physics changes.

## Status

Complete; committed.

## Implementation commit

`dd15909896b0bb9a3a0590a6025700a9ec938f7d`
