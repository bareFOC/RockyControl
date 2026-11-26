# Design

## Context

Measured (thumbv8m.main-none-eabihf, release, DEFMT_LOG=info): hardware build `usb-net,imu-lsm6dsv,i3c-servo-bus` = 153,424 B flash / 32,100 B SRAM. The 16,036-byte URDF (`model_store.rs:21-30`) is embedded in `.rodata` and served by an optional UDP task (`usb_net.rs:112-149,214`) that the control path never uses. The `bench-loopback` feature is a no-op flag — `ServoBusSlot` always owns a `BenchLoopbackBus` fallback (`servo_bus.rs:115-134,150-166`). Release profile uses `lto = "thin"`; `lto = "fat"` measured −13,144 B flash with no SRAM cost.

## Goals / Non-Goals

**Goals:** remove redundant MCU model data and service; make bench plant compile-time optional; fat LTO; preserve control/wire/replay behavior.

**Non-Goals:** changing `rk-control`/`rk-proto`/`rk-sim`, wire version, replay fixtures, defmt levels, fast-math, or algorithm gating.

## Decisions

1. `model_store.rs`: delete `EMBEDDED_URDF_BYTES`, `EMBEDDED_MODEL_INFO`, `bundled_rocky()` and the `flash_model_store_streams_full_urdf_with_exact_crc32` test. Keep `FlashModelStore::new`, `info()`, `handle_request`/`handle_cobs_datagram` and their generic tests (still used by the sim and host). Update module docs to say the MCU no longer bundles a URDF.
2. `usb_net.rs`: delete the `udp_model_service_task` and its spawn; remove `USB_NET_MODEL_UDP_PORT`-related imports/constants if now unused. Document that the app owns model data.
3. `firmware/Cargo.toml`: `lto = "thin"` → `"fat"` in `[profile.release]`; keep all other settings.
4. `bench-plant` feature in `rk-fw-core/Cargo.toml`; `rk-rp2350/Cargo.toml`: `bench-loopback = ["rk-fw-core/bench-plant"]`. Gate `BenchLoopbackBus`, the `bench` field and fallback branches in `servo_bus.rs` with `#[cfg(feature = "bench-plant")]`; ensure hardware builds that register hooks still compile without it, and that `ServoBusSlot::read_feedback`/`write_command` have a safe no-op path when no hooks and no bench plant are present (or leave existing default behavior with a documented fallback). Add `rk-esp32p4` feature parity if needed for compilation.
5. Update affected docs (`firmware/README.md`, `README.md`) to note MCU model pull is unavailable; app owns URDFs.

## Risks

- Removing `bundled_rocky()` breaks sim tests that call it? Grep shows sim uses `FlashModelStore::new` and its own `include_bytes!` — verify with full workspace tests.
- Gating the bench fallback must not break default CDC builds (which rely on it). Verify default build and tests.
- Fat LTO changes codegen but not f32 semantics; rerun replay/golden tests to confirm bit-exactness.
