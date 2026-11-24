# Tasks

> Delegation note: each numbered group is a self-contained unit with clear file ownership, intended for one subagent. Do not edit tasks.md checkboxes — parent handles that.

## 1. Async LSM6DSV SPI (OWNER: driver agent)

- [x] 1.1 Convert `firmware/rk-rp2350/src/imu_lsm6dsv.rs` from `Spi::new_blocking`/`blocking_transfer` to Embassy async SPI (`Spi::new` with Async mode, `write_read_async`/`transfer_async`), keeping manual CS via `Output`, the LSM6DSV register constants, FIFO burst parsing via `Lsm6dsvFilter::update_from_fifo_burst`, and `IMU_SLOT` publication with `LSM6DSV_OK`. Verify by `cargo check --manifest-path firmware/rk-rp2350/Cargo.toml --target thumbv8m.main-none-eabihf --features imu-lsm6dsv`.
- [x] 1.2 Confirm the task still publishes at ≥200 Hz (one burst per 5 ms tick) and yields during transfers; add a code comment. Verify by the same check and a brief report of the transfer path.

## 2. I3C critical-section refactor (OWNER: bus agent; parallel to Group 1)

- [x] 2.1 Refactor `firmware/rk-rp2350/src/i3c_simplefoc.rs` so `I3cTransport::transaction` and all 21-joint I/O occur OUTSIDE any `critical_section::with`; use short critical sections only to snapshot command/transport/tracker state and to record per-joint results. Preserve per-joint timeout counters, degraded_mask, bus_error, and edge-triggered `defmt::warn!`. Verify by `cargo check --manifest-path firmware/rk-rp2350/Cargo.toml --target thumbv8m.main-none-eabihf --features i3c-servo-bus`.
- [x] 2.2 If the external PIO driver needs an await point, add an `embassy_sync::channel` handoff to a dedicated `i3c_io` task (add dependency if used); otherwise document why the synchronous trait remains safe outside CS. Report the decision.

## 3. Feature guards + cleanups (OWNER: integration agent; parallel)

- [x] 3.1 Add `compile_error!` for `uart-servo-bus + i3c-servo-bus` in `firmware/rk-rp2350/src/main.rs`; consolidate `Irqs` cfg predicates for readability; document `bench-loopback` override semantics in `firmware/README.md`. Verify by thumb checks of conflicting and non-conflicting feature sets.
- [x] 3.2 Remove no-op `copper`/`copper-defmt`/`copper-std` feature aliases from `firmware/rk-fw-core/Cargo.toml` and refresh stale copper-rs comments in `firmware/rk-rp2350/memory.x`. Verify `cargo check --manifest-path firmware/Cargo.toml -p rk-fw-core` and RP2350 default check still pass.
- [x] 3.3 (Optional) Promote `control_task` to an `InterruptExecutor` for priority isolation; if the boilerplate is risky, keep thread-mode and record as deferred. Either way verify the control loop still runs at 200 Hz with watchdog feeding intact.

## 4. Integration verification (OWNER: integrator; depends on all groups)

- [x] 4.1 Run `cargo test -p rk-proto -p rk-control -p rk-cu-nodes -p rk-sim -p rk-model -p rk-link -p rk-viewport -p rk-ui -p rk-mcp` and `cargo test --manifest-path firmware/Cargo.toml -p rk-fw-core`; verify zero failures.
- [x] 4.2 Run RP2350 `cargo check` for default, imu-bno055, uart-servo-bus, imu-lsm6dsv, i3c-servo-bus, and usb-net; plus the negative conflict test; verify all pass / fail as expected.
- [x] 4.3 Run `sh scripts/lint.sh` and `openspec validate embassy-guidelines --strict`; verify both pass.
