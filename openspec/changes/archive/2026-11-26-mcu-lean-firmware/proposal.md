# Proposal

## Why

The MCU currently embeds a 16,036-byte URDF in flash and runs a UDP model service (port 7778) purely as a convenience download for the app. The control kernel never parses or uses it, and all model/URDF data is moving into the app. Removing this, plus two low-risk build changes, shrinks the production hardware firmware from ~153 KB / 32 KB SRAM to ~119 KB / 29 KB SRAM without touching the real-time control path, wire protocol, or replay determinism.

## What Changes

- Remove the MCU's embedded URDF and the UDP model service task: delete `EMBEDDED_URDF_BYTES`/`EMBEDDED_MODEL_INFO`/`bundled_rocky()` and their unit test from `rk-fw-core/src/model_store.rs`; delete `udp_model_service_task` and its spawn from `rk-rp2350/src/usb_net.rs`. The app uses its own bundled/preset/uploaded URDFs; `mcu_sync_model` degrades gracefully with a clear error.
- Switch the firmware release profile from `lto = "thin"` to `lto = "fat"` (keep `opt-level = "s"`, `codegen-units = 1`, `panic = "abort"`). Measured −13.1 KB flash across all builds.
- Gate the bench plant out of hardware firmware builds: add a `bench-plant` feature to `rk-fw-core`, make `rk-rp2350`'s `bench-loopback` enable it, and wrap `BenchLoopbackBus` and its `ServoBusSlot` fallback in `#[cfg(feature = "bench-plant")]`. Production builds save ~1.5 KB flash / 0.8 KB SRAM.

## Capabilities

### Modified Capabilities
- `firmware-integration-health`: leaner firmware with no redundant model data and optional bench hardware.
- `unified-control-application`: unchanged control loop, wire protocol and replay semantics.

## Impact

Files: `firmware/rk-fw-core/src/model_store.rs`, `firmware/rk-fw-core/src/servo_bus.rs`, `firmware/rk-fw-core/Cargo.toml`, `firmware/rk-rp2350/src/usb_net.rs`, `firmware/rk-rp2350/Cargo.toml`, `firmware/Cargo.toml`. UI/MCP docs note that MCU model pull is unavailable (app-owned models). No changes to `rk-control`, `rk-proto`, `rk-sim`, wire format or replay fixtures.

Deferred: runtime flash-loaded model blobs, defmt log level tuning, debug symbol stripping, per-robot profile URDF embedding (separate `multi-robot-profiles` change).
