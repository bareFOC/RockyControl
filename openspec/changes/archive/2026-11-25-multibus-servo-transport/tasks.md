# Tasks

> Delegable. Group 1 first; Groups 2/3 in parallel; Group 4 integrates.

## 1. New rk-multibus crate (OWNER: comms agent; FIRST)
- [x] 1.1 Create `crates/rk-multibus/` (Cargo.toml + lib.rs) with deps: `barefoc-master` (path `../bareFOC/barefoc-master`, default-features off), `embedded-hal 1.0`, `rk-proto`/`rk-control` workspace; add to root workspace. Verify `cargo check -p rk-multibus`.
- [x] 1.2 Implement `joint_map.rs`: node addr `0x20+joint`, register↔RawServoCommand/Feedback mapping (0x120/0x130/0x134/0x138/0x144/0x148/0x14C + temperature offset), unit conversions, mode mapping. Verify unit tests `cargo test -p rk-multibus`.
- [x] 1.3 Implement `transport.rs`: `embedded_hal::i2c::I2c` adapter over an external PIO transaction boundary (trait/closure), error mapping; `health.rs` ported bus-health. Add loopback test using `embedded-hal-mock`/RegMap. Verify `cargo test -p rk-multibus`.

## 2. Firmware migration (OWNER: firmware agent; depends on 1)
- [x] 2.1 Rewrite `firmware/rk-rp2350/src/i3c_simplefoc.rs` transaction loop to use `barefoc-master::ControlTableClient` via `rk-multibus` I2C adapter; keep CS discipline + per-joint degraded/timeout + `defmt::warn!`. Verify thumb checks for `i3c-servo-bus`.
- [x] 2.2 Delete old codec (`pack_i3c_joint_cmd`, `unpack_i3c_joint_reply`, `simple_crc8`, `encode_joint_cmd`) from `firmware/rk-fw-core/src/i3c_bus.rs`; keep bus-health; update tests. Retire UART placeholder or mark legacy. Verify `cargo test --manifest-path firmware/Cargo.toml -p rk-fw-core`.
- [x] 2.3 Document new wire format + register profile in `firmware/README.md`; update openspec archive references if needed.

## 3. Simulator migration (OWNER: sim agent; depends on 1, parallel to 2)
- [x] 3.1 Route `rk-sim::ServoBusPort` through `rk-multibus` register client against an in-memory register server/mock I2C; plant publishes/consumes the same registers instead of raw struct fields. Verify `cargo test -p rk-sim`.
- [x] 3.2 Add end-to-end test proving sim wire encoding: command written to register table → plant acts → feedback registers populated → mapped back to RawServoFeedback. Verify `cargo test -p rk-sim`.

## 4. Verification (OWNER: integrator; depends on all)
- [x] 4.1 Full host suite + firmware-core + RP2350 thumb feature matrix + WASM check + lint + strict validate.
- [x] 4.2 Document servo-side deployment assumption (matching bareFOC multibus firmware + register offsets) in README.
