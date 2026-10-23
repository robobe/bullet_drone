# TODO

- [ ] **Centralized simulation telemetry recorder** — implement after simulation telemetry schemas and YAML topic-selection configuration.
  - Ground-truth UDP live preview is implemented in [tools/telemetry_sender](../tools/telemetry_sender/README.md); general recording and other-topic collection remain future work.
  - Use one bounded queue per recorder, shared across enabled topics. Keep queue handling out of individual topic modules.
  - Submit immutable frames without blocking the simulation loop; correlate samples from the same step using drone ID, episode ID, step index and simulation time.
  - Centralize drop counters, worker lifecycle and shutdown handling in the recorder.
  - Serialize and batch file or network output in a background worker. Capture already-computed values without recomputing simulation effects.
  - On queue overflow, continue simulation, count dropped frames and mark the recording incomplete. Mandatory topics cannot be disabled, but queueing does not guarantee lossless delivery.
  - Verify topic correlation, queue overflow, worker failure reporting and cleanup. Measure capture/queue latency and real-time factor before choosing queue capacity or claiming performance guarantees.

This is future work. Adding this checklist does not implement telemetry schemas, configuration, collection or output.
