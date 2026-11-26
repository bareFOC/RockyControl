# Tasks

> Prerequisite: `multi-robot-profiles` landed on main. Run `cargo test` after each phase.

## 1 — URDF REP-103 Normalization & Joint Sign Verification
- [x] 1.1 Rotate `assets/robots/humanoid.urdf` (joint/link origins, inertial frames, visual/collision mesh origins, and `imu_mounting_rpy` in `assets/robots/humanoid.robot.toml`) from `-Y` forward to ROS REP-103 (`+X` forward, `+Y` left, `+Z` up).
- [x] 1.2 Audit and verify all 32 revolute joint axis sign directions (`l_knee_upper/lower`, `l_ankle_pitch`, `l_toe_pitch`, `waist_yaw/pitch/roll`, hips, shoulders, elbows, neck) against REP-103 right-hand-rule.
- [x] 1.3 Run `cargo run -p xtask -- codegen-robots`, rebuild the embedded `rk-model` `.zip` asset bundle, and update/verify `com_fk_golden_test.rs` and `rk-viewport` rendering.

## 2 — UVC / Balance / SLIP Tuning (`h_com ≈ 0.25 m`)
- [x] 2.1 Use `rk-mcp` (`experiment_run_named` / `run_experiment` + `quality_snapshot`) to sweep and tune `BalanceGains` (`h_com_m`, `hip_height_m`, `foot_half_length_m`, `foot_margin_m`, UVC step timing and capture gains for $\omega = \sqrt{g / h_{\text{CoM}}} \approx 6.2\text{ rad/s}$) and `RunningConfig`.
- [x] 2.2 Commit converged 28-DoF defaults in `rk-proto` / `rk-control` and verify `balance_reset_defaults` parity between host and kernel.

## 3 — Toe Push-Off & 2-Point Heel+Toe Contact Tuning
- [x] 3.1 Tune `toe_pitch` push-off timing/amplitude and 2-point heel+toe contact parameters in `rk-sim` across `walk_nominal` and `walk_shove` (including realistic sensor noise/latency).
- [x] 3.2 Verify single-support GRF, zero flight ticks, bounded foot slip, and shove recovery in `rk-sim` tests.

## 4 — 28-DoF Keyframes, Acceptance Bounds & Verification Gate
- [x] 4.1 Retune `default_recovery_library` (prone, supine, left-side, right-side) and `default_combat_library` in `crates/rk-model/src/studio.rs` for 28-DoF joint limits and coupled double-joints; verify `fall_recover` in simulation.
- [x] 4.2 Update `rk-mcp` / `rk-sim` acceptance bounds for the 28-DoF humanoid and regenerate `tests/experiments/walk_nominal.copper`, `walk_nominal.json`, `walk_shove.json`, and `fall_recover.json`.
- [x] 4.3 Run `cargo run -p xtask -- sim-verify` and `openspec validate "humanoid-sim-bringup" --strict`; archive and sync specs on completion.
