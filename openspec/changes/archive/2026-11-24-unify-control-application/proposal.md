# Proposal

## Why

The firmware and simulator currently maintain parallel copies of kernel-wrapper, transport, and replay logic (~400–500 duplicate lines). Worse, replay verification exercises the simulator's `DirectServoTask`, not the firmware's `ControlTask` — so firmware-only bugs escape simulation. Long-term maintainability and the goal that "if it works in simulation it works on hardware" require a single source of truth for the control application, with hardware drivers and the physics plant as the only swappable backends.

## What Changes

- **BREAKING (internal APIs, no wire change promised):** Make `rk-fw-core` the single host+MCU crate for the control application. Add it as a path dependency of the host workspace so sim/UI can use it.
- Delete `rk-cu-nodes::DirectServoTask`; use `rk_fw_core::ControlTask` as the only kernel wrapper in both firmware and simulation.
- Remove global mailbox coupling from `ControlTask::step()` by making wire-error counts and tick recording injectable/explicit, so the same wrapper is deterministic and callable from a fixed-dt simulation harness.
- Delete duplicate `rk-cu-nodes::ServoBusDriver`; implement the existing `rk_fw_core::servo_bus::ServoBusPort` trait on `SimulatedServoPlant` so sim and hardware share one bus contract.
- Unify COBS framing / host-link mailbox: use `rk_fw_core::transport` from sim and replace `rk-cu-nodes::HostLinkBridge`; delete the duplicate.
- Use `FlashModelStore` in sim instead of reimplementing `exchange_model_request`; delete duplicate dead message structs in `rk-fw-core` (~370 lines used only by size tests).
- Move `CopperListTick`/`CopperReplayLog`/`verify_deterministic_replay` from `rk-sim` into `rk-fw-core`, so replay verifies the exact wrapper firmware uses. Update UI/MCP replay call sites.
- Keep the fixed-dt synchronous simulation harness (no Embassy-on-std/WASM for the control path — async runtime timing would make replay nondeterministic). Embassy remains the hardware runtime only.

## Capabilities

### New Capabilities
- `unified-control-application`: Guarantees that firmware and simulation execute the same control application code path, with only I/O backends swapped, and that replay verification covers the firmware code path.

### Modified Capabilities
- `firmware-integration-health`: Rewiring the control application into `rk-fw-core` changes which crate provides the kernel wrapper and transport, tightening single-source-of-truth requirements.

## Impact

- **Crates:** `rk-fw-core` (single source; gains replay types + host-compatible API), `rk-cu-nodes` (gutted or removed), `rk-sim` (uses `rk-fw-core`), `rk-link`/`rk-ui`/`rk-mcp` (import path changes for replay), root `Cargo.toml` + firmware workspace manifests (dependency wiring).
- **Firmware board crates:** `main.rs` continues to call `ControlTask`; no behavior change.
- **Risk:** touching replay and transport affects verified tests; mitigate by keeping `DirectServoKernel` untouched and running the full shove/fall/replay suite after each step.
