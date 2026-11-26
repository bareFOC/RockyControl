# Design

## Context

`Kinematics/humanoid_description` defines the primary 28-DoF humanoid (32 URDF revolute joints: 3 waist, 3 neck/head, 2×(3 hip + 2 coupled knee + 2 ankle + 1 toe), 2×(3 shoulder + 2 coupled elbow), 1.3931 kg, Z-up, chest-mounted IMU, 36 STLs). Replacing Rocky Mk1 (21 DoF) requires widening `MAX_JOINTS` from 24 to 36 (`torque_mask: u64`, wire v7), representing 1:2 coupled double-joints, modeling heel+toe contact, transforming a chest IMU to the pelvis frame, and splitting 28 servos across up to 4 I3C buses. Consistent with `mcu-lean-firmware`, MCU firmware stays URDF-free and mesh-free.

## Goals / Non-Goals

**Goals:** single source of truth via `crates/rk-robot-profile`; `MAX_JOINTS = 36` (`u64` mask, wire v7); URDF-free MCU profiles; `xtask codegen-robots` with golden tests; 28-DoF humanoid (coupled double-joints, toe push-off, 3-DoF waist balance, chest IMU, 2-point foot contact, up to 4 I3C buses) + 8-DoF Striker.

**Non-Goals:** runtime flash-loaded profiles, const-generic wire arrays, backward-compatibility shims for Rocky Mk1 or wire v6, locomotion algorithms for <17-DoF robots.

## Decisions

1. **Acyclic crate layout (`rk-robot-profile`):** Move `LegGeometry`, `LinkMass`, `JointKin`, and `TorqueConstants` into `no_std` `crates/rk-robot-profile`. `rk-control` and `rk-multibus` depend on `rk-robot-profile` directly (no compatibility re-exports per `AGENTS.md`). `RobotProfile` holds `name`, `joint_count` (actuators), `urdf_joint_count`, `JointIndices` (with `Option<u8>` roles including `l/r_toe_pitch`, `waist_pitch/roll`), `mass`, `joint_kin` (each with `actuator_index` + `mimic_scale`), `motor_to_joint_ratio: [f32; MAX_JOINTS]` (`0.5` for double-knees/elbows), `pelvis_index`, `left/right_foot_index`, `left/right_toe_index`, `imu_link_index`, `imu_mounting_rpy`, `leg_geometry`, `foot_half_length_m`, `foot_margin_m`, `h_com_m`, `torque_constants`, `max_torque_nm`, `max_velocity_rad_s`, `servo_endpoints: [ServoEndpoint; MAX_JOINTS]`, and `default_limits`.
2. **Protocol & capacity (`MAX_JOINTS = 36`, wire v7):** Bump `MAX_JOINTS`, `MAX_IMPACT_JOINTS`, and `MAX_SERVO_JOINTS` to `36`; widen `torque_mask` from `u32` to `u64`; bump `PROTOCOL_VERSION` to `7` (still fits `MAX_COBS_FRAME_BYTES = 1024`). Regenerate `.copper` replay fixtures.
3. **Coupled double-joints & calibration:** Control space uses anatomical total bend `θ_bend`. `calibrated_to_raw` / `raw_to_calibrated` apply `motor_to_joint_ratio` (`0.5` for double-knees/elbows) so `zero_offset_rad` stays in raw motor encoder space. `com_fk` and `KinematicRig`/`rk-viewport` map each URDF half-joint via `actuator_index` and `±0.5` scale.
4. **Chest IMU compensation:** `com_fk` exposes `R_pelvis_to_imu(q_waist)` from the `base_link -> Waist -> Backbone -> Chest_1` chain + `imu_mounting_rpy`. `KinematicEstimator` rotates chest IMU orientation/rates into the pelvis frame (`R_world_to_pelvis = R_world_to_imu * R_pelvis_to_imu^T`) before UVC/ukemi/fall-cutoff.
5. **UVC, SLIP & 2-point foot contact:** `LegJointAngles` adds `toe_pitch` for stance/push-off/swing; UVC feeds pitch/roll balance into `waist_pitch` / `waist_roll`; SLIP torque overlay uses `profile.joint_indices.is_slip_leg_joint(i)` including toes. `rk-sim` uses `com_fk` heel/sole + toe-tip frames for 2-point foot contact.
6. **Codegen & assets:** Rename `Revolute 1..34` in `humanoid.urdf` to industry-standard `l_`/`r_` `snake_case` (`l_knee_upper/lower`, `l_toe_pitch`, `waist_yaw/pitch/roll`, etc.) and move to `assets/robots/humanoid.urdf` + `humanoid.robot.toml` (and `striker_biped.urdf` + `striker_biped.robot.toml`). Embed the 36 STLs + URDF in `rk-model` as a compressed `.zip` for desktop/WASM. `xtask codegen-robots` emits `crates/rk-robot-profile/src/generated/*.rs`; add `com_fk_golden_test.rs`; delete `mass_table.rs` and hardcoded `JOINT_KINEMATICS`.
7. **URDF-free firmware & 4-bus I3C:** `firmware/rk-profile-humanoid` and `firmware/rk-profile-striker` export `pub const PROFILE: RobotProfile` with **no** URDF bytes or `model_info`. `rk-rp2350` selects `robot-humanoid` / `robot-striker` via mutually exclusive features with `compile_error!`; `i3c_simplefoc.rs` supports up to 4 registered `PioI3c` transports dispatched by `PROFILE.servo_endpoints[i]`. Keep `rk-esp32p4` compiling against `RobotProfile` + `MAX_JOINTS = 36`.

## Risks

- Regenerating all `.copper` fixtures for wire v7 + 28-DoF humanoid: verify determinism with fresh `ControlTask` replay runs.
- Chest-IMU de-rotation sign/frame conventions: verify with unit tests tilting `waist_pitch/roll` while pelvis stays level.
- 36-STL `.zip` load time on WASM: measure `rk-ui` WASM build and startup decode time.
