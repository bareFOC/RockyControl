# Tasks

> Delegation note: each numbered group is a self-contained unit with clear file ownership, intended for one subagent. Do not edit tasks.md checkboxes — parent handles that.

## 1. Shared messages + control task (OWNER: firmware-core agent)

- [x] 1.1 In `firmware/rk-fw-core/src/copper.rs`, remove all cu29 trait impls (`CuTask`, `CuBridge`, `Freezable`, `Encode/Decode/Reflect` derives, `#[copper_runtime]`-generated glue) while KEEPING the plain message structs (`HostCmdMsg`, `ServoFeedbackMsg`, `ControlStepMsg`) and their `from_host`/`to_host`/`to_servo_cmd`/`to_telemetry` conversions. Optionally rename the module to `control.rs` and update `lib.rs`. Preserve the `size_of` guardrail test. Verify by `cargo check --manifest-path firmware/Cargo.toml -p rk-fw-core`.
- [x] 1.2 Add a plain `ControlTask` helper in `rk-fw-core` that owns a `DirectServoKernel` and exposes `step(now_ms, dt, cmd: Option<&HostToMcu>, fb: &RawServoFeedback) -> KernelStepOutput`, replacing the Copper task's `process()` body. Verify by a unit test that calls it with a default command/feedback and asserts non-panicking output.
- [x] 1.3 Remove cu29 dependencies and codegen from `firmware/rk-fw-core/Cargo.toml` (drop `copper`, `copper-defmt`, `copper-std` features and `cu29` dep; keep `rk-control`/`rk-proto` etc.) and delete `cu29_build::setup()` from firmware build scripts as appropriate. Verify by `cargo check --manifest-path firmware/Cargo.toml -p rk-fw-core`.

## 2. RP2350 Embassy runtime (OWNER: board agent; depends on Group 1)

- [x] 2.1 Rewrite `firmware/rk-rp2350/src/main.rs`: remove `extern crate alloc`, `HEAP_MEM`/`LockedHeap`/`init_heap`, `RobotClock`, `NoopLogger`/`NoopSectionStorage`, `#[copper_runtime]`, and `running_app.run_one_iteration()`. Declare and spawn Embassy tasks: a high-priority `control` task with `Ticker::every(CONTROL_PERIOD_MS)` that calls `ControlTask` using `embassy_sync` channels (RX host commands, TX telemetry, servo command/feedback), plus keep existing USB/IMU tasks. Keep the 250 ms hardware watchdog fed only after clean iterations and the GPIO25 LED driven from `current_led_mode()`. Add `embassy-sync` dependency. Verify by `cargo check --manifest-path firmware/rk-rp2350/Cargo.toml --target thumbv8m.main-none-eabihf` for all feature sets.
- [x] 2.2 Adapt `usb_cdc.rs` and `usb_net.rs` to feed the new host-link channels instead of `HOST_LINK_MAILBOX` (preserve COBS framing, 64-byte chunking, UDP 7777, ZLP handling, device descriptors). Verify by the same thumb checks including `usb-net`.
- [x] 2.3 Remove `cu29`/`cu29-build` from `firmware/rk-rp2350/Cargo.toml`, remove `build.rs` cu29 setup (keep memory.x copy/linker flags), and update `firmware/README.md` architecture section to describe the Embassy-only runtime. Verify by full feature-matrix check.
- [x] 2.4 Confirm all new tasks/channels are actually wired (no dead code): grep for `spawner.spawn` of each task and channel send/receive call sites. Report findings.

## 3. ESP32-P4 scaffold alignment (OWNER: board agent; parallel to Group 2, depends on Group 1)

- [x] 3.1 Update `firmware/rk-esp32p4/src/main.rs` to the same Embassy-only shape: remove `#[copper_runtime]`/cu29, use the shared `ControlTask` with a 200 Hz Ticker, keep the documented TODOs for USB/WD/LED (do not implement full bring-up), stop swallowing `run_one_iteration()` errors since that call is gone. Remove cu29 deps from `rk-esp32p4/Cargo.toml`/`build.rs`. Verify by `cargo check --manifest-path firmware/rk-esp32p4/Cargo.toml` (note: target toolchain may be absent — report clearly if it cannot run).

## 4. Host replay preservation + integration verification (OWNER: integrator; depends on all groups)

- [x] 4.1 Confirm host replay is untouched: `cargo test -p rk-sim -p rk-cu-nodes -p rk-ui -p rk-mcp` passes, including `verify_deterministic_replay` and the shove/fall tests. Rename user-visible ".copper" wording to "tick log" in `rk-ui/src/config_screen.rs` and `rk-mcp/src/schema.rs` ONLY if desired — behavior unchanged; record decision.
- [x] 4.2 Run the full host suite `cargo test -p rk-proto -p rk-control -p rk-cu-nodes -p rk-sim -p rk-model -p rk-link -p rk-viewport -p rk-ui -p rk-mcp` and `sh scripts/lint.sh`; verify zero failures.
- [x] 4.3 Run `cargo check` for RP2350 default, `imu-bno055`, `uart-servo-bus`, `imu-lsm6dsv`, `i3c-servo-bus`, and `usb-net` feature sets; verify all pass.
- [x] 4.4 Run `openspec validate simplify-mcu-runtime --strict`; verify valid.

## Workflow follow-up

- Review the simplified firmware before archiving.
- A follow-up change will address remaining Embassy guideline items (LSM6DSV async SPI, feature guards, ESP32-P4 bring-up) on the simplified base.
