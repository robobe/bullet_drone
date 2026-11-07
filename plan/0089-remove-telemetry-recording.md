# Remove Bullet Drone telemetry and recording output

Status: Implemented and verified. Implementation commit: `9fdb8375300743d92f83f77b8a072bc070207d8b`.

## Goal

Remove Bullet Drone's simulation telemetry schemas, optional PlotJuggler sender,
and publisher stream used by the external-RC recorder.

## Implementation steps

1. Remove `TelemetrySettings`, `SimulationConfig.telemetry`, YAML parsing and
   profile blocks. Keep strict rejection of legacy `telemetry:` YAML sections.
2. Remove the schema package, PlotJuggler sender, position-square CLI option,
   and the publisher's `analysis/physics/*` queue/output and callers.
3. Remove feature-specific tests, update sensor tests to validate observation
   readings directly, and remove stale feature documentation and references.
4. Preserve sensor behavior, identity fields, renderer pose transport, and all
   sibling workspace projects; verify the focused and full Bullet Drone suites.

## Interfaces and scope

Breaking removal of the Bullet Drone telemetry schema/config API, optional
sender CLI, and analysis stream. Work is limited to this repository. Existing
recordings and sibling `mcap_bagger`/FlightLens projects remain untouched; future
sibling recordings no longer receive Bullet Drone analysis topics. YAML files
containing `telemetry:` fail the strict unknown-section check.

## Verification

- `uv run --locked pytest -q` — 559 passed, 3 skipped.
- Focused config, sensor and Iris profile checks — 106 passed.
- Focused position-square, renderer, ZMQ and external-RC checks — 56 passed.
- Position-square `--help` no longer lists `--telemetry-config`; the focused CLI test confirms the removed option is rejected.
- Legacy YAML `telemetry:` is rejected, and the remaining checked-in profiles load in the passing suite.
- `git diff --check` and workspace `uv lock --check` passed.

## Implementation commit

Pending.
