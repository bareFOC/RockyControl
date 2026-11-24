# Tasks

> Delegation note: each numbered group is a self-contained unit with clear file ownership, intended for one subagent. Do not edit tasks.md checkboxes — the parent agent handles that.

## 1. Host test fixes (OWNER: control agent)

- [x] 1.1 Gate COM/DCM validity bits on IMU availability in `crates/rk-control/src/estimator.rs` (`valid_mask` assembly ~lines 222–233) so a sample with `imu.available == false` does not report valid CoM/DCM; add a unit test for the IMU-outage path; verify by `cargo test -p rk-control` passing all tests including `v1_valid_mask_marks_grf_and_momentum_unavailable`.
- [x] 1.2 Investigate and fix the `rk-sim` shove-recovery regression (`mcu_uvc_balance_catch_steps_combat_shove_and_unassisted_falls` ends in E-Stop instead of standing). Inspect `crates/rk-control/src/estimator.rs` swing-foot modulation (~209–217) and `crates/rk-control/src/uvc.rs` DCM hint application (~568–578). Apply the minimal correction (swing gain / $T_s$ feedforward and/or support-relative DCM subtraction) that restores recovery, keeping fall cutoff and latch intact. Verify by `cargo test -p rk-sim` passing all tests, and re-run `cargo test -p rk-control` to ensure no regression.

## 2. Firmware test + feature fixes (OWNER: firmware agent; parallel to Group 1)

- [x] 2.1 Fix the LSM6DSV FIFO settling off-by-one in `firmware/rk-fw-core/src/lsm6dsv.rs` (~236–240, 302–325) so a 5-sample burst crossing the 6-sample settling threshold marks the sensor available on the expected burst; verify by `cargo test --manifest-path firmware/Cargo.toml -p rk-fw-core` passing all tests including `fifo_burst_averages_five_960hz_samples_and_discards_settling`.
- [x] 2.2 Update `firmware/rk-rp2350/src/servo_uart.rs` command/feedback literals to supply the new fields from the extended `RawServoCommand`/`RawServoFeedback` structs: `command_mode`, `raw_velocity_rad_s`, `torque_limit_nm` (commands) and `i3c_timeout_count`, `imu_source_mask`, `velocity_rad_s` (feedback). Use neutral/safe defaults so behavior is unchanged. Verify by `cargo check --manifest-path firmware/rk-rp2350/Cargo.toml --target thumbv8m.main-none-eabihf --features uart-servo-bus` succeeding.
- [x] 2.3 Correct IMU source attribution: change `firmware/rk-rp2350/src/imu_bno055.rs` to use `publish_with_source(..., BNO055_OK, ...)` instead of default `publish()`; verify by `cargo check --manifest-path firmware/Cargo.toml -p rk-fw-core` and a focused test asserting the source mask.

## 3. Board driver wiring (OWNER: board integration agent; depends on Group 2)

- [x] 3.1 Add `imu-lsm6dsv` and `i3c-servo-bus` features to `firmware/rk-rp2350/Cargo.toml` (defaults unchanged). Verify by `cargo check --manifest-path firmware/rk-rp2350/Cargo.toml --target thumbv8m.main-none-eabihf` with and without the new features.
- [x] 3.2 Create `firmware/rk-rp2350/src/imu_lsm6dsv.rs`: configure SPI1 from the LSM6DSV constants (WHO_AM_I 0x70 with 2–3 retries, CTRL3 BDU/IF_INC/SW_RESET, CTRL6 gyro FS+LPF1, CTRL7 LPF1 enable, CTRL8 accel FS, FIFO 960 Hz continuous, INT1), spawn an Embassy task that burst-reads the FIFO and calls `Lsm6dsvFilter::update_from_fifo_burst`, then publishes `ImuState` to `IMU_SLOT` with the `LSM6DSV_OK` source mask. Gate BNO055 spawn off when this feature is enabled in `main.rs`. Verify by thumb-target check with `--features imu-lsm6dsv` and a brief config note in `firmware/README.md`.
- [x] 3.3 Create `firmware/rk-rp2350/src/i3c_simplefoc.rs`: instantiate `I3cBusTracker`, register `ServoBusHooks` via `SERVO_BUS_SLOT.register_hooks` whose read/write functions use the existing `encode_joint_cmd` / `unpack_i3c_joint_reply` and call `record_timeout` on failed transactions; add `defmt::warn!` on transition to degraded. Wire from `main.rs` under `i3c-servo-bus`. Verify by thumb-target check with `--features i3c-servo-bus` and a test or code path proving the hooks are registered (not dead code).

## 4. Integration verification (OWNER: integrator; depends on all groups)

- [x] 4.1 Run full host suite `cargo test -p rk-proto -p rk-control -p rk-cu-nodes -p rk-sim -p rk-model -p rk-link -p rk-viewport -p rk-ui -p rk-mcp` and `cargo test --manifest-path firmware/Cargo.toml -p rk-fw-core`; verify zero failures.
- [x] 4.2 Run `cargo check --manifest-path firmware/rk-rp2350/Cargo.toml --target thumbv8m.main-none-eabihf` for default, `imu-bno055`, `uart-servo-bus`, `usb-net`, `imu-lsm6dsv`, and `i3c-servo-bus` feature sets; verify all succeed.
- [x] 4.3 Run `sh scripts/lint.sh` and `openspec validate fix-p0-audit-regressions --strict`; verify both pass.

## Workflow follow-up

- Review the implemented fixes before archiving.
- Archive the change with the OpenSpec archive workflow once CI is green.
