# Betaflight MSP backend

Status: implemented and verified.
Implementation commit: `19d3247`

The agreed cross-project plan is [here](../../los_tracking/docs/betaflight-backend-plan.md).

Reuse the existing FDM codec with separate pure packing, UDP transport and motor
adaptation. Keep physics/sensors independent of MSP and process supervision.
Preserve internal control, wind and sensor APIs. Validate unit, UDP and live SITL
behavior, then full application/Godot flow. See the shared plan for interfaces,
defaults, exclusions and verification results.
