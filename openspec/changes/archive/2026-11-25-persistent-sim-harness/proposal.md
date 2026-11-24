# Proposal

## Why

Every `SimTransport::step_fixed` call currently creates a fresh `DeterministicHarness`/raw Embassy executor, but `embassy_time::MockDriver` is process-global. Repeated tool calls leave stale wakers in the global timer queue, causing a `cordyceps` stack assertion that panics the MCP HTTP thread (poisoning its state mutex) and makes live MCP replay unstable. Unit tests pass only because they serialize/reset the clock. This change makes the simulation harness persistent for the transport's lifetime.

## What Changes

- `SimTransport` owns one `DeterministicHarness` (and one spawned control task over its `SimRobotIo`) created at construction, reused across all `step_fixed` calls.
- Support re-queueing host commands and retrieving telemetry on the persistent task via shared state (channels/slots), without re-spawning the Embassy task.
- Keep the raw kernel `ControlTick` recording path intact for bit-exact replay; keep plant-augmented telemetry in returned frames.
- Remove test-only clock resets / test lock workarounds that existed solely to hide the global-executor churn (where safe).

## Capabilities

### New Capabilities
- `persistent-simulation-executor`: The simulation owns a long-lived deterministic Embassy executor, allowing repeated fixed-step calls within one process without corrupting global timer state.

### Modified Capabilities
- `mcp-deterministic-stepping`, `embassy-sim-runtime`: rely on persistent executor semantics.

## Impact
- `crates/rk-sim/src/lib.rs` (`SimRobotIo`, `CopperSimRunner`), `crates/rk-link/src/lib.rs` (`SimTransport`), possibly `crates/rk-harness/src/lib.rs` (support re-spawning or persistent APIs).
- Risk: self-referential ownership (`'static` task borrowing IO) requires careful raw-pointer/ leaked-box design like the existing per-call harness — but done once at construction.
