# Design

## Context

See `proposal.md`. Today `rk-fw-core` (firmware workspace) and `rk-cu-nodes`/`rk-sim` (host workspace) contain parallel kernel wrappers, bus traits, transport helpers, model-store logic, and replay implementations. `rk-fw-core` already has `default = ["std"]` and runs its tests on the host, so it is the natural single source of truth. Replay currently verifies the simulator's wrapper, so firmware wrapper bugs are invisible to simulation — the core problem this change fixes.

## Goals / Non-Goals

**Goals:** one shared control application in `rk-fw-core`; simulation and hardware use identical wrapper/transport/bus/replay code; only the physics plant and hardware drivers differ; firmware wrapper changes are caught by replay.

**Non-Goals:** running Embassy's executor in simulation (fixed-dt sync harness stays); changing `DirectServoKernel` behavior; changing wire framing semantics; ESP32-P4 bring-up; adding new control features.

## Decisions

### 1. Make `rk-fw-core` a host dependency
Add `rk-fw-core` as a path dependency in the root workspace (e.g. to `rk-sim`), with `std` feature for host builds. It stays `no_std`-compatible for firmware. Re-export `ControlTask`, `ServoBusPort`, transport types, and replay types from its lib root.

### 2. Pure `ControlTask::step`
Change `step(now_ms, dt_sec, cmd, fb, wire_errors) -> KernelStepOutput` so wire error counts come from an explicit parameter rather than a global mailbox read. Tick recording (for replay) happens in a separate explicit `record_tick()` method or via a returned `CopperListTick`-like snapshot — no hidden global state. Firmware's `control_task` passes mailbox counts in; sim passes its own (usually zero).

### 3. One bus trait
Delete `rk-cu-nodes::ServoBusDriver`. Implement `rk_fw_core::servo_bus::ServoBusPort` for `SimulatedServoPlant` so `read_feedback`/`write_command` are shared contract. Hardware continues to use `ServoBusHooks`/the I3C/UART drivers through `SERVO_BUS_SLOT`.

### 4. One transport + model store
Move/keep COBS framing and mailbox in `rk_fw_core::transport`; have `rk-link::SimTransport` and `rk-sim` use it instead of `rk-cu-nodes::HostLinkBridge`. Replace `rk-sim::exchange_model_request` with `FlashModelStore::handle_request`. Delete the now-unused dead message structs in `rk-fw-core/copper.rs` (keep only those actually used; simplify size tests).

### 5. Replay in the shared crate
Move `CopperListTick`, `CopperReplayLog`, `ReplayVerification`, and `verify_deterministic_replay` into `rk-fw-core` (they depend only on `rk-control`/`rk-proto` types). Update `rk-sim` to call the shared verifier on a `ControlTask`; update UI/MCP imports.

### 6. What `rk-cu-nodes` becomes
Either delete the crate entirely (remove from workspace) or reduce it to a thin alias module re-exporting shared types for any downstream code that still imports it. Prefer deletion per AGENTS.md (no back-compat shims).

## Risks / Trade-offs

- [Cross-workspace path dependency] → Cargo allows path deps outside workspace root; verify host and firmware builds resolve the same source version (single copy on disk).
- [Replay format change] → tick log JSON structure may change field names; no back-compat needed; update UI/MCP together.
- [Wire error count injection] → firmware must pass real counts; sim passes zero; verify telemetry still shows correct values.
- [Determinism] → keep fixed-dt sync harness; do not introduce Embassy executor timing into replay path.
