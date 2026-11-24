# Design

## Context

See `proposal.md` for motivation. The audit of HEAD `43bf5ee` found: (a) three failing tests (`rk-control` valid_mask, `rk-sim` shove recovery, `rk-fw-core` FIFO settling); (b) `uart-servo-bus` feature broken by extended `RawServoCommand`/`RawServoFeedback` structs; (c) `lsm6dsv.rs` and `i3c_bus.rs` compiled into `rk-fw-core` but never instantiated in `rk-rp2350`; (d) BNO055 publishing via a default `publish()` that tags samples as `LSM6DSV_OK`. The 1024-byte wire format, pre-COBS CRC-32, protocol version, torque-limit field, and valid_mask already exist and are correct — **no wire-format changes in this fix**.

## Goals / Non-Goals

**Goals:** green CI; board drivers actually wired in; correct IMU source attribution; isolated per-joint bus faults; a stable shove-recovery simulation.

**Non-Goals:** implementing the full lumped-mass CoM FK (P1, not P0); closed-loop torque/impedance control (v2); ESP32-P4 BSP bring-up; vision or learned policies; changing the wire protocol.

## Decisions

### 1. Validity gating, not test weakening
`estimator.rs` sets COM/DCM validity bits unconditionally. Fix the implementation to require `imu.available` (and the source mask) before setting them, rather than changing the failing test's expectation. Rationale: the test encodes the intended contract from the spec; a sample with no live sensor must not report valid CoM/DCM. Add a unit test for the IMU-outage path.

### 2. FIFO settling off-by-one — make the gate burst-aware
Current gate checks `settled_samples < 6` at burst entry, then consumes the whole 5-sample burst, so crossing the boundary needs a third burst. Fix by counting individual samples within the burst and marking `available` as soon as the cumulative settled count reaches the threshold *within* the current burst, while still averaging the remaining valid samples. Preserve the 6-sample settling constant.

### 3. Sim shove regression — fix the controller, not the test
The test expects recovery to Drive; the new DCM path ends in E-Stop. **Investigation result (implementation finding):** the actual root cause was NOT the swing-foot gain or DCM frame. Commit `43bf5ee` removed the capture-point rate lead (`pitch_eff + 0.30·pitch_rate`, `roll_eff + 0.26·roll_rate`) from `integrate_uvc_spherical` and the ankle proportional term (`-0.22·raw_pitch`, `-0.22·raw_roll`), leaving an underdamped I-only stance integrator that accumulated lean across support swaps until the fall cutoff latched. The fix restores those two terms with rates passed from `step_with_dcm`. Fall cutoff detection, FallTuck duration, TILT_FALL_CUTOFF latch, and all clamps remain untouched. The test is the oracle; do not relax safety.

### 4. Update the UART stub to the extended structs
Populate the new fields in `servo_uart.rs`: `command_mode`, `raw_velocity_rad_s`, `torque_limit_nm` in commands; `i3c_timeout_count`, `imu_source_mask`, `velocity_rad_s` in feedback. Use neutral/safe values (position mode, torque limit at max, zero velocity) so behavior is unchanged. This is a compile fix, not a functional upgrade.

### 5. Correct IMU source attribution
Use `IMU_SLOT.publish_with_source(..., BNO055_OK, ...)` in `imu_bno055.rs` instead of the default `publish()` (which tags `LSM6DSV_OK`). When `imu-lsm6dsv` is enabled, the BNO055 task is not spawned at all; the LSM6DSV SPI task publishes with `LSM6DSV_OK`. This satisfies the modified `fall-detection` spec.

### 6. Wire the drivers behind new features
Add `imu-lsm6dsv` and `i3c-servo-bus` features to `rk-rp2350/Cargo.toml`. Add `imu_lsm6dsv.rs` (SPI1 config, INT1/DRDY or FIFO-burst polling task, `verify_lsm6dsv16x_id` with 2–3 retries, register writes from the existing constants, publish to `IMU_SLOT` with `LSM6DSV_OK`) and `i3c_simplefoc.rs` (instantiate `I3cBusTracker`, register `ServoBusHooks` that call `encode_joint_cmd`/`unpack_i3c_joint_reply`, call `record_timeout` on failed transactions, emit `defmt::warn!` on degradation). Keep default features unchanged (`usb-cdc`, `bench-loopback`) so existing behavior is preserved.

### 7. I3C logging and scope
`I3cBusTracker::record_timeout` currently increments counters silently. Add `defmt::warn!` on transition to degraded (not on every transient timeout). The tracker's wire CRC8 encoding is kept; the RP2350 PIO low-level transaction code already exists outside this repo per user confirmation, so the board module adapts to it via the hooks rather than reimplementing PIO.

## Risks / Trade-offs

- [Swing gain change could over-step on large pushes] → Re-run the full shove suite; keep clamps (±0.18 m) intact.
- [New SPI/LSM6DSV task competes with CPU budget] → FIFO burst at 200 Hz (one DMA read per tick) keeps overhead low; verify with thumb target check.
- [Feature proliferation] → Defaults unchanged; new features are opt-in and documented in `firmware/README.md`.
- [I3C adapter depends on external PIO driver] → The hook module must degrade gracefully if that driver is absent; the feature is off by default.
