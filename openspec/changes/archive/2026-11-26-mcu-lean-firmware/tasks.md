# Tasks

> Delegable. Verify green after each. Final: archive + sync + commit + push.

## 1 — Remove MCU model data/service (OWNER: firmware agent; FIRST)
- [x] 1.1 Delete embedded URDF/model info/bundled_rocky + its test from `rk-fw-core/src/model_store.rs`; delete UDP model service task + spawn from `rk-rp2350/src/usb_net.rs`; update docs.
- [x] 1.2 Verify `cargo test --workspace`, `cargo test --manifest-path firmware/Cargo.toml -p rk-fw-core`, thumb checks for all feature sets, MCP acceptance tests (sim `mcu_sync_model` with staged blob still passes).

## 2 — Fat LTO + bench gate (OWNER: build agent; after 1)
- [x] 2.1 `firmware/Cargo.toml` release `lto = "fat"`; add `bench-plant` feature in rk-fw-core, wire rk-rp2350 `bench-loopback`, gate bench plant with cfg. Verify default + hardware builds and tests; confirm replay fixtures still bit-exact.
- [x] 2.2 Measure sizes with `arm-none-eabi-size`; report flash/SRAM deltas vs baseline (153,424 / 32,100 hardware): **119,188 B flash (−34,236 B) / 29,160 B SRAM (−2,940 B)**; `arm-none-eabi-nm` confirms 0 `FlashModelStore` / `BenchLoopbackBus` / `EMBEDDED_URDF` symbols.

## 3 — Finish (OWNER: integrator)
- [x] 3.1 Full gate: `cargo run -p xtask -- sim-verify` (all 8 steps: no-std, workspace-tests, wasm32-check, fw-core-tests, RP2350 thumb matrix, MCP acceptance experiments), `openspec validate "mcu-lean-firmware" --strict`; archive + sync.
