# Periodic render profile announcement

## Goal

Ensure Godot receives the drone render profile even when its ZeroMQ subscriber
connects after publisher startup.

## Implementation steps

1. Send the existing profile packet with each periodic render configuration
   announcement instead of only once per publisher lifetime.
2. Keep the native adapter's stream-id deduplication unchanged.
3. Extend the TCP publisher test to verify repeated profile announcements.

## Interface changes

No schema or API changes. The existing `render/drone_1/profile` packet is
repeated at the existing configuration interval.

## Verification

Run the focused ZeroMQ render publisher tests and the native adapter tests.

## Scope and defaults

This fixes the startup slow-joiner race without adding a handshake or changing
Godot fallback behavior.

## Status

Implemented; focused publisher and adapter checks passed.

## Implementation commit

`53b96b1`
