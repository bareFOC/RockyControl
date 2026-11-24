# Tasks

> Delegation note: each numbered group is a self-contained unit with clear file ownership, intended for one subagent. Do not edit tasks.md checkboxes — parent handles that. No backward compatibility needed; delete obsolete code rather than shimming.

## 1. Shared crate foundation (OWNER: core agent)

- [x] 1.1 Make `rk-fw-core` usable from the host workspace: add it as a path dependency (with `std` feature) where needed (root Cargo.toml / rk-sim / rk-link), keep it `no_std`-compatible for firmware. Verify `cargo check -p rk-fw-core` (host) and `cargo check --manifest-path firmware/Cargo.toml -p rk-fw-core` both pass.
- [x] 1.2 Refactor `ControlTask::step` in `firmware/rk-fw-core/src/copper.rs` to take explicit wire-error counts instead of reading `HOST_LINK_MAILBOX` globally; add an explicit per-tick snapshot/recording hook (return or record a `CopperListTick`-like value) with no hidden global state. Update the firmware `control_task` call site in rk-rp2350/rk-esp32p4 main.rs to pass mailbox counts. Verify thumb checks + host checks pass.
- [x] 1.3 Delete dead message structs from `rk-fw-core/src/copper.rs` (keep only those actually used by firmware/host; simplify or remove size-only tests accordingly). Verify both host and firmware checks pass.

## 2. Move replay into shared crate (OWNER: replay agent; depends on Group 1)

- [x] 2.1 Move `CopperListTick`, `CopperReplayLog`, `ReplayVerification`, and `verify_deterministic_replay` from `rk-sim`/`rk-cu-nodes` into `rk-fw-core` (a new `replay` module), adapting them to operate on `ControlTask`. Re-export from lib.rs. Verify `cargo test -p rk-fw-core` (host) passes.
- [x] 2.2 Update `rk-sim`, `rk-ui`, `rk-mcp` to import replay types/verifier from `rk-fw-core`; remove old definitions from `rk-sim`/`rk-cu-nodes`. Verify `cargo test -p rk-sim -p rk-ui -p rk-mcp` passes, including the deterministic replay and shove/fall tests.
- [x] 2.3 Confirm replay now verifies the shared `ControlTask` (add/updated test asserting a wrapper behavior change is detected); report the evidence.

## 3. Unify bus, transport, model store (OWNER: io agent; parallel to Group 2, depends on Group 1)

- [x] 3.1 Delete `rk-cu-nodes::ServoBusDriver`; implement `rk_fw_core::servo_bus::ServoBusPort` on `SimulatedServoPlant` in `rk-sim` so simulation and firmware share the trait. Verify sim tests pass.
- [x] 3.2 Replace `rk-cu-nodes::HostLinkBridge` usage in sim/link with `rk_fw_core::transport` types; ensure SimTransport uses shared COBS framing/mailbox. Verify `cargo test -p rk-link -p rk-sim` passes.
- [x] 3.3 Replace `rk-sim::exchange_model_request` with `FlashModelStore::handle_request`; delete the duplicate. Verify sim tests pass.
- [x] 3.4 Delete `rk-cu-nodes` crate entirely (remove workspace member and path deps) once no code imports it. Verify full host build passes.

## 4. Integration verification (OWNER: integrator; depends on all groups)

- [x] 4.1 Run full host suite `cargo test -p rk-proto -p rk-control -p rk-sim -p rk-model -p rk-link -p rk-viewport -p rk-ui -p rk-mcp` and `cargo test --manifest-path firmware/Cargo.toml -p rk-fw-core`; verify zero failures.
- [x] 4.2 Run RP2350 thumb `cargo check` for default, imu-bno055, uart-servo-bus, imu-lsm6dsv, i3c-servo-bus, usb-net; verify all pass.
- [x] 4.3 Confirm no duplicate kernel-wrapper / bus-trait / replay definitions remain (grep/report); confirm sim replay test uses shared `ControlTask`.
- [x] 4.4 Run `sh scripts/lint.sh` and `openspec validate unify-control-application --strict`; verify both pass.
