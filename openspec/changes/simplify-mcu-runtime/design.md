# Design

## Context

See `proposal.md`. Current MCU firmware uses copper-rs as a thin generated wrapper: `#[copper_runtime]` generates one `DirectServoCuTask` plus `HostLinkBridge`/`ServoBusBridge`, called once per 5 ms from an Embassy `Ticker` loop. All actual I/O is Embassy (USB tasks, IMU tasks, SPI/UART). The glue — `HOST_LINK_MAILBOX`, `IMU_SLOT`, `SERVO_BUS_SLOT` (critical-section mutexes), a 64 KiB buddy heap, `RobotClock::new_with_rtc` with a spin closure, and `NoopLogger` — exists only to bridge sync copper-rs into async Embassy. Research confirmed copper-rs has no official Embassy integration and its replay/logging features are unused on the MCU.

## Goals / Non-Goals

**Goals:** one scheduler (Embassy) on the MCU; remove all cu29 runtime glue; preserve behavior and wire format; keep host replay/determinism untouched; eliminate the Embassy-guideline violations at their source.

**Non-Goals:** changing `DirectServoKernel` or any control algorithm; changing the wire protocol; implementing on-hardware `.copper` logging; ESP32-P4 full bring-up (USB/WD/LED remain scaffold TODOs); changing sim/UI/MCP replay features.

## Decisions

### 1. Async task topology
- `host_link_rx` task: owns USB-CDC/NCM RX, decodes frames, sends `HostToMcu` into an `embassy_sync::channel` (capacity 2, latest-wins semantics preserved by dropping the old value on full).
- `host_link_tx` task: receives telemetry from a channel, encodes and writes to USB (existing CDC/NCM tasks adapted).
- `imu` task(s): existing BNO055/LSM6DSV tasks publish `ImuState` to a shared slot. Because `ImuState` is `Copy` and tiny, keep a critical-section slot (`IMU_SLOT`) — this is a sync boundary for a single value, not a scheduler bridge; no `embassy_sync` needed.
- `servo_bus` task: owns the physical bus (bench loopback / UART / I3C). Receives commands from a channel, performs transactions, sends `RawServoFeedback` back.
- `control` task: high-priority Embassy task with `Ticker::every(5ms)` that drains the RX channel, reads the IMU slot + servo feedback, calls `DirectServoKernel::step(...)`, and posts commands/telemetry to the TX channels.

### 2. Use `embassy_sync` where truly async↔async
Replace `HOST_LINK_MAILBOX` and the servo command/result path with `embassy_sync::channel::Channel` (or `Signal` for telemetry) — the idiomatic Embassy mechanism. Keep the small critical-section slots only for single-value sensor snapshots read by the sync control function (these are not awaited inside a critical section). **No hardware I/O inside any critical section** — resolving the P0 I3C anti-pattern structurally.

### 3. High-priority control loop
Use embassy-rp's interrupt executor (`InterruptExecutor`) or the main executor with the control task given the shortest `Ticker` period; the deterministic 5 ms cadence remains. Since `DirectServoKernel::step()` is fast and allocation-free, no separate scheduler authority is needed. The hardware watchdog (250 ms) stays as the backstop.

### 4. Drop allocator and cu29
Remove `extern crate alloc`, the `HEAP_MEM` static, `buddy_system_allocator`, `NoopLogger`, `RobotClock`, and all `cu29` dependencies/codegen (`cu29-build`, build.rs setup). Firmware becomes fully no-alloc. `firmware/rk-fw-core/src/copper.rs` is rewritten to plain message structs + a `ControlTask` helper (no traits/codegen); the file may be renamed `control.rs`.

### 5. Preserve replay in host only
`rk-cu-nodes::DirectServoTask` and `rk-sim::CopperReplayLog` remain the replay backbone. No changes to `rk-protocol`, `rk-control`, `rk-model`, `rk-ui`, `rk-mcp`.

## Risks / Trade-offs

- [Firmware has few automated tests; regression risk] → keep `DirectServoKernel` byte-for-byte unchanged; verify with `cargo check` on both MCUs + bench-loopback shove test in sim; preserve telemetry fields exactly.
- [Removing cu29 loses future declarative-graph option] → documented in proposal; can be reintroduced later on a clean Embassy base if needed.
- [embassy_sync channel capacity/backpressure] → capacity 2 with overwrite-oldest for commands/deadman semantics; telemetry via `Signal` (latest wins), matching current mailbox behavior.
- [ESP32-P4 scaffold divergence] → apply the same task shape but keep the existing TODOs for USB/WD/LED; the change only removes cu29 there.
