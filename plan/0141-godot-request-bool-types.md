# Explicit Godot request-control boolean types

## Goal and steps
Fix main.gd's reported parser error by explicitly typing request availability and request busy status as bool. Use request_busy to distinguish request activity from the existing camera-capture busy member. Review nearby request-client expressions for the same inference pattern.

## Interfaces and scope
No wire or behavior changes. Renderer main controls only.

## Verification
Source inspection only. No tests or builds by user instruction; manual rebuild pending.

## Status
Implemented; uncommitted.

## Implementation commit
Pending.
