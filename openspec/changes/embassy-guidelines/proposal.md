# Proposal

## Why

With copper-rs removed, the firmware is now a pure Embassy application — but an Embassy-guideline audit found remaining anti-patterns: LSM6DSV SPI uses blocking transfers in an async task (~1.8 ms/tick CPU stall), I3C transactions run inside a `critical_section` with interrupts disabled (starving USB and the watchdog), servo-bus features can silently override each other, and stale cu29 comments remain in the linker script. These are the last structural obstacles to a clean, low-jitter 200 Hz control loop for a fighting/acrobatic humanoid.

## What Changes

- Convert the LSM6DSV driver on RP2350 from blocking SPI to **async/DMA** (`Spi::new` + `write_read_async`), matching the BNO055's async I2C; consume the 960 Hz FIFO in one burst per 5 ms tick and keep publishing `ImuState` with `LSM6DSV_OK`.
- Refactor the I3C servo adapter so hardware transactions happen **outside** `critical_section::with`; keep short critical sections only for snapshotting the command/transport pointer and recording per-joint results. Add `embassy_sync` channel/Signal handoff if needed. Preserve per-joint timeout isolation and edge-triggered `defmt::warn!` logging.
- Add `compile_error!` guards for conflicting servo-bus features (`uart-servo-bus` + `i3c-servo-bus`); simplify `Irqs` cfg predicates into a single readable alias; document that `bench-loopback` hooks are overridden when a hardware bus is enabled.
- Optional: promote the 200 Hz control task to an `InterruptExecutor` for priority isolation over USB (deferred if it requires excessive boilerplate).
- Clean stale copper-rs comments in `firmware/rk-rp2350/memory.x` and remove no-op `copper*` feature aliases from `rk-fw-core/Cargo.toml`.

## Capabilities

### New Capabilities
<!-- None — these are internal firmware quality/behavior improvements. -->

### Modified Capabilities
- `firmware-integration-health`: Adds requirements that hardware I/O must not run inside critical sections, that async drivers use Embassy's async/DMA APIs, and that conflicting bus features are compile-time rejected.

## Impact

- **Firmware:** `firmware/rk-rp2350/src/{imu_lsm6dsv.rs, i3c_simplefoc.rs, main.rs}`, `firmware/rk-rp2350/{Cargo.toml, memory.x}`, `firmware/rk-fw-core/Cargo.toml` (no-op feature cleanup).
- **Host crates:** none. Replay/determinism and wire protocol unchanged.
- **Risk:** async SPI and I3C refactors affect timing-critical paths; validate with thumb-target checks and existing bench/sim tests.
