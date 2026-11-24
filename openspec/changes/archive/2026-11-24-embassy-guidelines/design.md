# Design

## Context

See `proposal.md`. After `simplify-mcu-runtime`, the firmware is pure Embassy: a 200 Hz `control_task` calls `ControlTask::step()`, USB/IMU tasks feed it via existing mailboxes. An Embassy-guideline audit found two genuine I/O-boundary violations (blocking SPI in an async task, I3C transactions inside `critical_section`) plus feature-guard/documentation gaps. Copper-rs determinism/replay is host-side and unaffected.

## Goals / Non-Goals

**Goals:** eliminate blocking I/O from async tasks; remove hardware I/O from critical sections; make conflicting bus features a compile error; clean stale comments/no-op features; keep timing and behavior otherwise identical.

**Non-Goals:** changing control algorithms, wire protocol, replay, or the mailbox architecture; full ESP32-P4 bring-up; migrating `HOST_LINK_MAILBOX` to `embassy_sync` (deliberately kept — see below).

## Decisions

### 1. Async/DMA SPI for LSM6DSV
Use `embassy_rp::spi::Spi::new(...)` (Async mode) instead of `Spi::new_blocking`, with `write_read_async`/`transfer_async` for the 2-byte register reads and the up-to-224-byte FIFO burst. Manual CS via `Output` (Embassy SPI does not auto-drive CS). Keep the `Lsm6dsvFilter` parser and `IMU_SLOT` publication unchanged; only the transport changes. Yields the executor during transfers (~1.8 ms → CPU available to USB).

### 2. I3C transactions outside critical sections
`i3c_simplefoc.rs` currently holds `I3C_STATE.borrow_ref_mut(cs)` for the whole 21-joint transaction loop. Refactor: (a) short critical section to snapshot the current command + transport pointer + tracker state; (b) perform `transport.transaction(...)` calls with interrupts enabled; (c) short critical section to call `record_reply`/`record_timeout` and update `degraded_mask`. If the external PIO driver needs an await point, define an `embassy_sync::channel` handoff to a dedicated `i3c_io` task; otherwise keep the synchronous trait but outside CS. Preserve per-joint counters and edge-triggered `defmt::warn!`.

### 3. Feature guards
Add `compile_error!("uart-servo-bus and i3c-servo-bus are mutually exclusive")` near existing guards in `main.rs`. Keep `imu-bno055` vs `imu-lsm6dsv` and `usb-cdc` vs `usb-net` guards. Consolidate the duplicated `Irqs` cfg predicates into a single `#[cfg(...)]` alias comment. Document that `bench-loopback` is overridden by any registered hardware bus hooks.

### 4. Control-loop priority (optional)
If low effort, move `control_task` to embassy-rp's `InterruptExecutor` for preemption over USB. If it requires an NVIC/IRQ/StaticCell boilerplate that risks correctness, keep thread-mode (jitter unchanged from the old inline loop) and record as deferred.

### 5. Cleanups
Remove no-op `copper`/`copper-defmt`/`copper-std` feature aliases from `rk-fw-core/Cargo.toml` now that board manifests no longer request them. Refresh stale cu29 comments in `memory.x` (linker layout unchanged). Keep `HOST_LINK_MAILBOX`/`SERVO_BUS_SLOT` — replacing them with `embassy_sync` is not required for guideline compliance and would churn working code.

## Risks / Trade-offs

- [Async SPI changes timing] → validate FIFO burst still publishes ≥200 Hz; keep parser unchanged so behavior is deterministic.
- [I3C refactor with external PIO driver boundary] → the `I3cTransport` trait stays; only call-site locking changes; keep a fallback stub so faults remain isolated.
- [InterruptExecutor complexity] → optional, deferred if risky; do not destabilize the control loop.
