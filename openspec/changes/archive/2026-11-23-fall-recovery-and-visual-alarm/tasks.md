# Tasks

> Delegation note: each numbered group below is a self-contained unit with clear file ownership and is intended to be assigned to one subagent. Groups 1–3 touch disjoint file sets and may run in parallel only after Group 1 lands (protocol fields are consumed by Groups 2–4). Verify with the command in each task.

## 1. Protocol: full-orientation `ImuState` (OWNER: proto/firmware agent)

- [x] 1.1 Extend `ImuState` in `crates/rk-proto/src/lib.rs` with `yaw_rad: f32`, `yaw_rate_rad_s: f32`, and `gravity: [f32; 3]`, keeping all derives and `Default` working; verify by `cargo check -p rk-proto`.
- [x] 1.2 Add finiteness checks for all new IMU fields to `decode_mcu_cobs` in `crates/rk-proto/src/lib.rs` and update the round-trip test literal to include the new fields; verify by `cargo test -p rk-proto` passing including frame-length < 512 assertion.
- [x] 1.3 Update every `ImuState` construction site to supply the new fields: `firmware/rk-fw-core/src/imu.rs` (slot init + tests), `firmware/rk-fw-core/src/copper.rs` (flattened `ServoFeedbackMsg`/`ControlStepMsg` fields and all `From`/`to_telemetry` conversions at ~208–317, 423–427), `firmware/rk-rp2350/src/servo_uart.rs`, `crates/rk-sim/src/lib.rs`, `crates/rk-control/src/uvc.rs` tests; verify by `cargo check --workspace` (host) and `cargo check --manifest-path firmware/Cargo.toml -p rk-fw-core`.
- [x] 1.4 Extend `ImuSlot::publish` validation in `firmware/rk-fw-core/src/imu.rs` to reject non-finite yaw/gravity and add a test for a NaN gravity sample retaining last good state; verify by `cargo test --manifest-path firmware/Cargo.toml -p rk-fw-core`.
- [x] 1.5 Document the **BREAKING** telemetry change in `README.md` and `firmware/README.md` (lockstep host+firmware rebuild), and add the new BNO055 register constants (`0x2E` gravity, fallback `0x20` quaternion) in `firmware/rk-rp2350/src/imu_bno055.rs`; verify by reading the docs and `cargo check --manifest-path firmware/rk-rp2350/Cargo.toml --target thumbv8m.main-none-eabihf`.

## 2. Sensor + sim full orientation (OWNER: firmware/sim agent; depends on Group 1)

- [x] 2.1 Decode heading from `eul_buf[0..2]` and gyro-Z from `gyr_buf[4..6]` in `firmware/rk-rp2350/src/imu_bno055.rs`, and read the gravity vector from register `0x2E` (page 0, 6-byte little-endian i16, 0.01 m/s² per LSB) into the published `ImuState`; verify by `cargo check --manifest-path firmware/rk-rp2350/Cargo.toml --target thumbv8m.main-none-eabihf` and a defmt log of a sample showing non-zero gravity magnitude ~9.81 on hardware.
- [x] 2.2 Add fallback gravity derivation from quaternion `0x20` or Euler if `0x2E` read fails, per design risk mitigation; verify by a unit test or simulated I2C failure path showing valid gravity output.
- [x] 2.3 Add `yaw_rad` (and optional synthetic yaw-rate) to `SimulatedServoPlant` in `crates/rk-sim/src/lib.rs`, zero it in `reset_upright()`, and publish a gravity vector derived by rotating world-down `(0,-9.81,0)` by the inverse of the tilt quaternion used in kinematics; verify by `cargo test -p rk-sim` with a new test asserting gravity matches forward tilt.
- [x] 2.4 Apply world-Y yaw in `KinematicRig::compute_link_transforms_with_root` in `crates/rk-model/src/kinematics.rs` (`yaw_rot * tilt_rot * base_rot`), thread the new argument through `compute_link_transforms_with_tilt` and the `solve_limb_ik` call site in `crates/rk-viewport/src/lib.rs`; verify by `cargo test -p rk-model -p rk-viewport`.

## 3. Fall detection & classification (OWNER: control agent; depends on Group 1)

- [x] 3.1 Add a `FallOrientation` enum (`Prone`, `Supine`, `SideLeft`, `SideRight`, `Unknown`) and a pure `classify_fall(&ImuState) -> FallOrientation` function in `crates/rk-proto/src/lib.rs` (no_std) that rotates gravity into the body frame and picks the dominant horizontal component; verify by `cargo test -p rk-proto` with unit tests for all four directions and the unavailable/unknown case.
- [x] 3.2 Wire the classifier into the existing fall path so that when `TILT_FALL_CUTOFF` latches in `DirectServoKernel::step` (`crates/rk-control/src/lib.rs`) or `UvcController::step` (`crates/rk-control/src/uvc.rs`) the classified orientation is captured in telemetry (extend `BalanceTelemetry` with an orientation code, or add a dedicated field); verify by `cargo test -p rk-control -p rk-proto` with a test driving each tilt direction and asserting the captured orientation.
- [x] 3.3 Add an MCP telemetry JSON field for fall orientation in `crates/rk-mcp/src/tools.rs` and verify `cargo check -p rk-mcp`.

## 4. Recovery keyframe library (OWNER: action-studio agent; depends on Group 3)

- [x] 4.1 Factor the `pose()` closure in `crates/rk-model/src/studio.rs` into a public `KinematicRig::pose_from_overrides()` helper that clamps overrides to URDF joint limits; verify by `cargo test -p rk-model` with a test overriding a joint beyond its limit and asserting the clamped result.
- [x] 4.2 Add `default_recovery_library(&rig) -> Vec<ActionMove>` with four multi-keyframe get-up moves (prone, supine, left-side, right-side), each transitioning fallen → push-off → kneel → stand → Guard, authored by joint name via the new helper; verify by a unit test asserting four moves exist, each has >= 4 keyframes, and every angle is within joint limits and calibration soft limits.
- [x] 4.3 Append the recovery library in both `StudioModel::new_default()` and `StudioModel::load_bundle()` so moves survive URDF reload; verify by `cargo test -p rk-model` with a test that reloads the bundled URDF and confirms all four moves are still selectable by name.
- [x] 4.4 Update MCP action schema descriptions in `crates/rk-mcp/src/schema.rs` to mention recovery moves (lookup is already dynamic) and verify `cargo check -p rk-mcp`.

## 5. Pulsing red fall alarm in viewport (OWNER: viewport/UI agent; depends on Group 3)

- [x] 5.1 Add `fall_event_active: bool` to `ViewportOptions` (and its `Default`) in `crates/rk-viewport/src/lib.rs`, add `fall_event_active: bool` and `time_sec: f64` to `ViewportFrameData` in `crates/rk-viewport/src/scene.rs`, and populate both in the frame-data literal inside `show()` using `ui.ctx().input(|i| i.time)`; verify by `cargo check -p rk-viewport`.
- [x] 5.2 In `SceneResources::render()` (`crates/rk-viewport/src/scene.rs`), add a highest-priority fall branch computing `pulse = 0.5 + 0.5*sin(time_sec * TAU * 2.0)` and setting mesh `albedo`/`emissive` red, overriding heat/selection tints; verify by a unit test (or documented visual check in sim) that with `fall_event_active = true` the material emissive red channel exceeds the non-fall value and varies with `time_sec`.
- [x] 5.3 Populate `fall_event_active` at both viewport call sites (`crates/rk-ui/src/control_screen.rs`, `crates/rk-ui/src/dashboard_screen.rs`) from `latest_telemetry.faults.contains(TILT_FALL_CUTOFF) && latest_telemetry.imu.available`; verify by `cargo check -p rk-ui` and a simulation push-to-fall visual check showing both viewports pulse red.

## 6. Recovery panel and consent-gated trigger (OWNER: UI/state-machine agent; depends on Groups 4, 5)

- [x] 6.1 Add fields to `RockyStudioApp` in `crates/rk-ui/src/lib.rs`: `fall_event_active: bool`, `fall_orientation: FallOrientation`, `auto_recovery_enabled: bool` (default false), `operator_confirmed: bool`, `recovery_state: RecoveryState`, and `fall_dwell_sec: f32`, initializing them in `new()`; verify by `cargo check -p rk-ui`.
- [x] 6.2 Implement `update_fall_watcher()` called in `update_frame()` immediately after `ingest_telemetry()` in `crates/rk-ui/src/lib.rs`: latch fall from `TILT_FALL_CUTOFF`, record orientation, require a configurable tilt-below-threshold dwell, and implement the multi-frame `RecoveryState` transitions `ClearingFault` → `ArmingDrive` → `PlayingGetUp` → `Completed` using `reset_estop()`, waiting for fault clear, `set_authority_mode(Drive)`, `walk_cmd.walk_enabled = false`, `play_move(idx)`; verify by a headless/sim integration test that a triggered recovery reaches `Completed` and that the fault-clear step is observed before Drive is entered.
- [x] 6.3 Enforce consent: autonomous start only when `auto_recovery_enabled && operator_confirmed`, abort on any re-latched E-Stop or new fall during playback, and clear `fall_event_active` only after stable upright; verify by tests covering (a) auto disabled → no recovery, (b) re-fall during get-up → torque cut and Idle, (c) successful completion → guard stance and cleared alarm.
- [x] 6.4 Add a top-right Recovery HUD panel in `crates/rk-ui/src/control_screen.rs` using spare `viewport_hud_rects[3]`, following the existing child-`Ui` pattern, showing fall status, detected orientation, an `auto_recovery_enabled` checkbox (off by default), an operator confirm control, and a Get Up button that triggers the same recovery sequence; verify by `cargo check -p rk-ui` and a manual sim check that the panel appears and controls change state.
- [x] 6.5 Document operator/auto-recovery behavior and safety limitations in `README.md`; verify by reading the updated section.

## 7. Integration verification (OWNER: integrator; depends on all groups)

- [x] 7.1 Run full host test suite `cargo test --workspace` and lint `sh scripts/lint.sh`, fixing any cross-crate fallout from the protocol change; verify all tests pass.
- [x] 7.2 Check both firmware targets: `cargo check --manifest-path firmware/Cargo.toml` and `cargo check --manifest-path firmware/rk-rp2350/Cargo.toml --target thumbv8m.main-none-eabihf` with both feature sets (`usb-cdc` default and `usb-net`); verify both succeed.
- [x] 7.3 Run an end-to-end simulation scenario: apply a forward shove → fall latches → classifier reports Prone → robot pulses red in both viewports → operator confirms Get Up → fault clears → Drive re-entered → Prone get-up motion plays → robot returns to Guard and alarm clears; verify by recording the scenario result (log or screenshot) in the change notes.
- [x] 7.4 Run `openspec validate fall-recovery-and-visual-alarm --strict` and verify no validation errors.

## Workflow follow-up

- Review the implemented change with the project owner before archiving.
- After review, archive the change using the OpenSpec archive workflow.
