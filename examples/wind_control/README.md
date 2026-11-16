# Godot request-controlled simulation

```bash
uv run --locked python external_render/build.py
uv run --locked --extra render python -m examples.wind_control.main
```

The example starts stopped. Submit an XYZ position using Scenario, configure wind using Wind, and press Start. Stop ends the run; Reset is stopped-only and prepares another run. Completion remains stopped until Reset or viewer closure. Closing the viewer or Ctrl+C cleans up resources.

The default profile external_render/configs/wind.json selects wind_transport.json. Its request_endpoint serves spawn, wind and command on TCP 5560. Full wind settings and XYZ-only spawn requests use validated dataclasses. Replies contain success only, or failure and reason. There are no getters, polling, request IDs or automatic retries. State/wind subscriptions and live indicators are deferred; panels send drafts and show call results.

See [message formats and lifecycle](../../docs/design/three-request-topics.md). Rebuild the native renderer for this breaking interface change. Tests and builds were not run during implementation.
