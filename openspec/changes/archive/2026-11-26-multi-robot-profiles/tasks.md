# Tasks

> Sequential phases; verify green after each. Final: archive + sync + commit + push.

## 1 — Protocol v7, `MAX_JOINTS = 36` & Profile Seam (OWNER: kernel agent; FIRST)
- [x] 1.1 Bump `MAX_JOINTS` / `MAX_IMPACT_JOINTS` / `MAX_SERVO_JOINTS` to `36`, widen `torque_mask` to `u64`, and bump `PROTOCOL_VERSION` to `7` across `rk-proto`, `rk-control`, `rk-multibus`, `rk-fw-core`, `rk-sim`, `rk-ui`, `rk-mcp`.
- [x] 1.2 Create `crates/rk-robot-profile` (`no_std`) and move `LegGeometry`, `LinkMass`, `JointKin`, and `TorqueConstants` into it; update `rk-control` and `rk-multibus` imports directly.
- [x] 1.3 Add `DirectServoKernel::new_with_profile(profile, cal)`, `RobotIo::profile()`, `motor_to_joint_ratio` scaling, chest-IMU pelvis de-rotation in `KinematicEstimator`, `toe_pitch` + `waist_pitch/roll` in UVC, toe-inclusive SLIP leg gating, and 2-point heel+toe contact in `rk-sim`.
- [x] 1.4 Verify `cargo test --workspace`, `cargo run -p xtask -- no-std`.

## 2 — Assets, Codegen & 3D Bundle (OWNER: tooling agent; after 1)
- [x] 2.1 Copy `Kinematics/humanoid_description` into `assets/robots/`, rename `Revolute 1..34` to `l_`/`r_` `snake_case`, author `humanoid.robot.toml` (with 4-bus `ServoEndpoint`s, `0.5` double-joint ratios, real motor limits) and `striker_biped.robot.toml`; pack `humanoid` URDF + 36 STLs as compressed `.zip` in `rk-model`.
- [x] 2.2 Add `xtask codegen-robots` emitting `crates/rk-robot-profile/src/generated/*.rs`; add `com_fk_golden_test.rs`; delete hand-maintained `mass_table.rs` and `JOINT_KINEMATICS`.
- [x] 2.3 Update `KinematicRig` / `rk-viewport` for coupled double-joint `mimic` visualization, update UI/MCP presets from profile metadata, and remove `rocky_mk1` assets/references.
- [x] 2.4 Regenerate `.copper` replay fixtures and verify golden tests + `cargo test --workspace` + `xtask sim-verify`.

## 3 — URDF-Free Firmware Profiles & 4-Bus I3C (OWNER: firmware agent; after 2)
- [x] 3.1 Add `firmware/rk-profile-humanoid` and `firmware/rk-profile-striker` (no URDF bytes); add `robot-humanoid` / `robot-striker` features with `compile_error!` guards on `rk-rp2350`; update `i3c_simplefoc.rs` for up to 4 I3C buses; keep `rk-esp32p4` compiling.
- [x] 3.2 Verify `cargo test --manifest-path firmware/Cargo.toml -p rk-fw-core` and thumb checks for default, hardware (`robot-humanoid`), and `robot-striker` feature sets.

## 4 — Finish (OWNER: integrator)
- [x] 4.1 Full gate: workspace x2, firmware-core, thumb matrix, WASM, Android check, sim-verify, strict validate.
- [x] 4.2 Archive + sync OpenSpec; commit and push.
