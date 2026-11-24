# Proposal

## Why

The MCU firmware embeds copper-rs (cu29) as a nominal single-task wrapper around `DirectServoKernel`, while all real scheduling, timing, USB, IMU, and watchdog work is done by Embassy. The two-scheduler glue (custom critical-section mailboxes, 64 KiB allocator, RobotClock calibration, NoopLogger) is the source of the recent P0 bugs and violates Embassy guidelines. An audit confirmed our "Deterministic Copper Replay" is hand-rolled JSON in `rk-sim`, entirely independent of copper-rs — so removing cu29 from the MCU costs no replay or determinism. For a fighting/acrobatic humanoid, a single-scheduler, low-jitter Embassy firmware is the better long-term foundation.

## What Changes

- **BREAKING (firmware architecture only, not wire protocol):** Replace the copper-rs `#[copper_runtime]` graph (1 task + 2 bridges) in both MCU crates with ordinary Embassy tasks and `embassy_sync` channels that call `DirectServoKernel::step()` directly.
- Remove the 64 KiB `buddy_system_allocator` heap, `NoopLogger`/`NoopSectionStorage`, `RobotClock`, and cu29 codegen from the MCU build; drop cu29 dependencies from `rk-fw-core` and both board crates.
- Move the control loop into a dedicated high-priority Embassy task driven by `embassy_time::Ticker` at 200 Hz; keep the host link and servo bus as async tasks feeding it via `embassy_sync::channel`/`Signal`.
- Keep `rk-cu-nodes::DirectServoTask` and the hand-rolled `rk-sim` replay/verification machinery **unchanged** — replay remains available in the desktop app and MCP.
- Preserve all safety behavior: watchdog, EStop, fault latches, soft/slew clamps, IMU fall detection, get-up recovery, telemetry wire format, and the 100 ms deadman.
- Keep `firmware/copperconfig.ron` only if still needed as a shared schema description for `rk-cu-nodes` (document its reduced role); otherwise remove from MCU build scripts.

## Capabilities

### New Capabilities
<!-- None — this is an internal architecture refactor with no externally observable behavior change. skip_specs: true is set. -->

### Modified Capabilities
<!-- None. Replay/determinism behavior and the wire protocol are unchanged. -->

## Impact

- **Firmware:** `firmware/rk-fw-core/src/copper.rs` (remove CuTask/CuBridge impls, keep message structs), `firmware/rk-rp2350/src/main.rs` (new Embassy tasks, remove heap/clock/logger), `firmware/rk-esp32p4/src/main.rs` (same shape, scaffold stays), `firmware/rk-fw-core/Cargo.toml`, both board `Cargo.toml`s, `build.rs` files, `firmware/copperconfig.ron` role.
- **Host crates:** none changed (`rk-cu-nodes`, `rk-sim`, `rk-ui`, `rk-mcp` keep their current replay code).
- **Risk:** the MCU firmware must be fully re-validated (tests are mostly host-side; firmware is validated by `cargo check` + bench loopback). Mitigate by keeping the pure `DirectServoKernel` untouched and moving only the orchestration.
