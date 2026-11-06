# Separate external-renderer startup methods

Status: Implemented and verified; committed. Implementation commit: `73e5b172f77e1b94309ce0cc204a4f2f344d8156`.

Goal: Extract Godot process startup and ZeroMQ communication startup into their
own focused methods in the external altitude-hold example's renderer.py.

Steps: keep port validation/log allocation in __enter__, extract _start_zmq and
_start_godot, preserve startup order and ExitStack cleanup registration, and add
a launch-failure regression to verify communication resources are released.

Interfaces/scope: Private method extraction only; public context-manager API,
commands, socket options, readiness checks and shutdown behavior stay unchanged.
No runtime/configuration features, dependencies, commit or push.

Verification: All 15 external-renderer tests passed, including the new launch-
failure case checking publisher-thread shutdown, socket/log closure and port
release after communication startup. Existing occupied-port, child-exit,
interruption, SHM ownership and mission cleanup checks passed. `git diff --check`
passed. No new Godot export or real graphical flight was run for this
behavior-preserving method extraction.

The user subsequently authorized committing and pushing all pending changes.
