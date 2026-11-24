# Proposal

## Why

An audit of HEAD `43bf5ee` (the kinematic estimator / DCM / LSM6DSV / I3C commit) found that CI is red: three unit tests fail, the `uart-servo-bus` firmware feature does not compile, and the two newly added hardware modules (LSM6DSV driver, I3C bus tracker) are never wired into the target firmware. A simulation balance test also regressed to E-Stop. These must be fixed before the P1a architecture can be trusted or extended.

## What Changes

- Fix the `rk-control` `valid_mask` test failure by gating COM/DCM validity bits on IMU availability at startup and when the sensor drops out.
- Fix the `rk-fw-core` LSM6DSV FIFO settling test off-by-one so a 5-sample burst crossing the 6-sample settling threshold is not wholly discarded and `available` is set on the expected burst.
- Fix the `rk-sim` balance regression where a shove ends in E-Stop instead of a recovered standing state (root cause found during implementation: the rate lead and ankle proportional term were removed from the stance integrator, leaving it underdamped; both are restored).
- Restore compilation of the `uart-servo-bus` feature by updating `servo_uart.rs` to the extended `RawServoFeedback` / `RawServoCommand` structs (new fields: `i3c_timeout_count`, `imu_source_mask`, `velocity_rad_s`, `command_mode`, `raw_velocity_rad_s`, `torque_limit_nm`).
- Correct IMU source attribution: BNO055 firmware SHALL publish with `BNO055_OK` source mask rather than the default `LSM6DSV_OK`, so telemetry never falsely claims the high-rate IMU is the active source.
- Wire the LSM6DSV driver and I3C bus modules into the RP2350 BSP behind explicit features (`imu-lsm6dsv`, `i3c-servo-bus`), with an SPI1 DRDY/FIFO-burst task publishing to `IMU_SLOT`, and an `i3c_simplefoc` board module registering `ServoBusHooks` with per-joint timeouts. **Not a protocol change** — only board integration.
- Remove BNO055 from the control path when `imu-lsm6dsv` is enabled (the magnetometer is disturbed by motor currents; gyro-Z integration supplies yaw).

## Capabilities

### New Capabilities
- `firmware-integration-health`: Guarantees that declared firmware features compile, that new board driver modules are actually instantiated on the target, and that per-joint bus faults are isolated without crashing the control loop.

### Modified Capabilities
- `fall-detection`: The "Full orientation telemetry" requirement's availability semantics are clarified — the IMU source tag must identify the actual live sensor (LSM6DSV vs BNO055), and a sample must not be marked available/valid until its settling period has completed.

## Impact

- **Host crates:** `rk-control` (estimator validity gating, possible DCM swing gain/frame fix), `rk-sim` (balance regression test), `rk-proto` (no wire change expected; source-mask semantics only).
- **Firmware:** `firmware/rk-fw-core/src/{imu.rs, lsm6dsv.rs, i3c_bus.rs}`, `firmware/rk-rp2350/src/{main.rs, servo_uart.rs, new imu_lsm6dsv.rs + i3c_simplefoc.rs}`, `firmware/rk-rp2350/Cargo.toml` (new features).
- **Docs (untracked reports):** `reports/ROCKY_ALGORITHM_ASSESSMENT.md` gravity-formula typo fix (cosmetic; done in a later cleanup, not in this change's code scope).
- **Risk:** The sim balance fix may require tuning DCM swing gain or making DCM support-relative; keep changes minimal and re-run the full shove test suite.
