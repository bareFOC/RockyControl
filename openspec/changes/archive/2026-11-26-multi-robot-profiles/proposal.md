# Proposal

## Why

The app accepts arbitrary URDFs for visuals and teach, but the control kernel, simulator plant, actuator constants, and I3C bus mapping were hardwired to Rocky Mk1 (21 DoF). Our target robot is a **28-DoF humanoid** (`Kinematics/humanoid_description`: 32 URDF revolute joints, 28 independent actuators with coupled double-knees/elbows, actuated toes, 3-DoF waist, and a chest-mounted IMU), alongside an 8-DoF Striker profile. To support both without duplicating firmware or breaking determinism, robot variation must become explicit compile-time data shared by firmware and simulator.

## What Changes

- Introduce a `no_std` `crates/rk-robot-profile` crate holding `RobotProfile` and the pure data types (`LegGeometry`, `LinkMass`, `JointKin`, `TorqueConstants`, `JointIndices`, `ServoEndpoint`, `CoupledJointMap`, IMU mounting metadata). `rk-control` and `rk-multibus` import these types directly from `rk-robot-profile` (no compatibility shims).
- Replace Rocky Mk1 completely as the primary robot with the **28-DoF humanoid** (`humanoid`), plus **8-DoF Striker** (`striker`) as the second profile:
  - Rename `Revolute 1..34` in `humanoid.urdf` to industry-standard `l_`/`r_` `snake_case` (`l_knee_upper`/`l_knee_lower`, `l_toe_pitch`, `waist_yaw/pitch/roll`, etc.) and pair with `.robot.toml` for motor/gearbox limits, `nm_per_amp`, and up to 4-bus I3C `ServoEndpoint { bus_id, node_addr }` mapping.
  - Embed the 36 STL meshes in `rk-model` as a compressed `.zip` so desktop and WASM stay self-contained; firmware stays **URDF-free and mesh-free** (consistent with `mcu-lean-firmware`).
- Bump `MAX_JOINTS` 24 → **36** (`MAX_IMPACT_JOINTS = 36`, `MAX_SERVO_JOINTS = 36`), widen `torque_mask` `u32` → **`u64`**, bump `PROTOCOL_VERSION` 6 → **7**, and regenerate `.copper` replay fixtures.
- Support 28-DoF mechanics in control and simulation:
  - Control space uses anatomical total bend (`θ_bend`); `RobotProfile` maps `0.5×` to the 1:1 motor/URDF half-joints for double-knees and double-elbows, and `com_fk` / `rk-model` / `rk-viewport` apply coupled joint transforms.
  - UVC drives `toe_pitch` push-off and feeds pitch/roll balance into `waist_pitch` / `waist_roll`; SLIP torque overlay includes toe joints; `rk-sim` models 2-point foot contact (heel + toe).
  - `KinematicEstimator` transforms the chest-mounted IMU reading back to the pelvis frame using `com_fk` waist kinematics + `imu_link_index`/`imu_mounting_rpy`.
- Add `xtask codegen-robots` to generate `const` tables from `assets/robots/*.urdf` + `*.robot.toml` into `crates/rk-robot-profile/src/generated/`, plus `com_fk_golden_test.rs`; delete hand-maintained tables.
- Add thin per-robot firmware profile crates (`firmware/rk-profile-humanoid`, `firmware/rk-profile-striker`) with `robot-humanoid` / `robot-striker` features and `compile_error!` guards on `rk-rp2350` (and keep `rk-esp32p4` compiling). Update `i3c_simplefoc.rs` for up to 4 I3C buses.
- Locomotion for <17 DoF (`striker`, 8-DoF) remains an explicit algorithmic limitation (position/velocity servo platform until a 4-DoF IK path exists).

## Capabilities

### New Capabilities
- `robot-profiles`: one compile-time `RobotProfile` supplies kinematics, mass, geometry, coupled-joint ratios, IMU placement, and multi-bus actuator data to firmware and simulator.

### Modified Capabilities
- `unified-control-application`: kernel accepts `&RobotProfile`, `MAX_JOINTS = 36` (`torque_mask: u64`, wire v7), chest-IMU pelvis compensation, coupled-joint ratio, toe/waist UVC & SLIP support.
- `simulator-fidelity`: plant reads profile geometry/actuators and models 2-point heel+toe contact.
- `firmware-integration-health`: board firmware selects one URDF-free robot profile at build time and supports up to 4 I3C servo buses.

## Impact

Files: new `crates/rk-robot-profile/**`, `firmware/rk-profile-*/**`, `assets/robots/**`; edits in `rk-proto` (`MAX_JOINTS=36`, `u64`, v7), `rk-control`, `rk-multibus`, `rk-fw-core`, `rk-sim`, `rk-model`, `rk-viewport`, `rk-ui`, `rk-mcp`, `rk-rp2350`, `rk-esp32p4`, `xtask`, `tests/experiments/*.copper`, specs.

Deferred: runtime flash-loaded profiles, const-generic kernel, locomotion algorithms for <17-DoF robots.
