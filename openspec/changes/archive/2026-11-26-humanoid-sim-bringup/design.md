# Design

## Context

`multi-robot-profiles` establishes the 28-DoF humanoid infrastructure (`crates/rk-robot-profile`, `assets/robots/humanoid.urdf` + `humanoid.robot.toml`, `xtask codegen-robots`, coupled double-joints, `toe_pitch` + `waist_pitch/roll` in UVC, 2-point heel+toe contact in `rk-sim`, and chest-IMU pelvis de-rotation). See [proposal.md](proposal.md) for motivation. Because the source CAD URDF was exported `-Y` forward and all gains, contact parameters, keyframes, acceptance bounds, and `.copper` fixtures originate from Rocky Mk1 (`h_com ≈ 0.165 m`), a dedicated simulation bringup and tuning pass is required before `xtask sim-verify` can pass on the 28-DoF humanoid (`h_com ≈ 0.25 m`, $\omega = \sqrt{g / h_{\text{CoM}}} \approx 6.2\text{ rad/s}$).

## Goals / Non-Goals

**Goals:**
- Normalize `assets/robots/humanoid.urdf` (and STL/joint/inertial transforms) to ROS REP-103 (`+X` forward, `+Y` left, `+Z` up) with verified joint axis signs, and regenerate `crates/rk-robot-profile/src/generated/` + `rk-model` `.zip` bundle.
- Tune `BalanceGains` and `RunningConfig` for `h_com ≈ 0.25 m` ($\omega \approx 6.2\text{ rad/s}$) using `rk-mcp` (`agentic-balance-tuning` sweeps).
- Tune `toe_pitch` push-off and 2-point heel+toe contact dynamics in `rk-sim` across `walk_nominal` and `walk_shove` (including realistic sensor profiles).
- Retune `default_recovery_library` and `default_combat_library` for 28-DoF limits, update `rk-mcp`/`rk-sim` acceptance bounds and `.copper` golden fixtures, and pass `cargo run -p xtask -- sim-verify`.

**Non-Goals:**
- Structural changes to `RobotProfile`, `PROTOCOL_VERSION`, or `MAX_JOINTS` (owned by `multi-robot-profiles`).
- Physical hardware bench bringup or `<17`-DoF `striker` locomotion algorithms.

## Decisions

1. **ROS REP-103 Normalization (`+X` forward, `+Y` left, `+Z` up):**
   Apply a `+90°` rotation about `+Z` to transform the `-Y`-forward CAD export into `+X`-forward REP-103 across `assets/robots/humanoid.urdf` (joint/link origins, inertial frames, visual/collision mesh origins, and `imu_mounting_rpy` in `humanoid.robot.toml`). Verify all 32 revolute joint axis signs against REP-103 right-hand-rule (`l_knee_upper/lower`, `l_ankle_pitch`, `l_toe_pitch`, `waist_yaw/pitch/roll`, hips, shoulders, elbows, neck), update `com_fk_golden_test.rs`, and run `xtask codegen-robots` + repack the `rk-model` `.zip` asset bundle.
   *Alternative:* Rotating only a root `base_link` dummy transform at runtime—rejected because `com_fk`, `rk-sim` contact frames, and `KinematicEstimator` expect link-local `+X` forward throughout the kinematic tree.
2. **Agentic UVC / Balance / SLIP Tuning via `rk-mcp`:**
   Use `rk-mcp` (`experiment_run_named`, `run_experiment`, `quality_snapshot`) to sweep `BalanceGains` (`h_com_m ≈ 0.25`, `hip_height_m`, `foot_half_length_m`, `foot_margin_m`, UVC step timing and capture gains for $\omega \approx 6.2\text{ rad/s}$) and `RunningConfig`, then commit the converged defaults in `rk-proto` / `rk-control`.
3. **Toe Push-Off & 2-Point Heel+Toe Contact Tuning:**
   Tune heel/toe contact stiffness, damping, friction, and `toe_pitch` push-off timing in `rk-sim` so `walk_nominal` and `walk_shove` (with realistic sensor noise/latency) exhibit smooth heel-to-toe load transfer, single-support GRF within 15% of body weight, zero flight ticks, and bounded slip.
4. **28-DoF Keyframes, Acceptance Bounds & Golden Fixtures:**
   Retune `default_recovery_library` (prone, supine, left-side, right-side) and `default_combat_library` in `rk-model` for 28-DoF joint limits and coupled double-joints; calibrate `rk-mcp` / `rk-sim` acceptance bounds for the 28-DoF geometry; regenerate `tests/experiments/walk_nominal.copper` and `*.json` fixtures; validate the full gate with `cargo run -p xtask -- sim-verify`.

## Risks / Trade-offs

- **URDF visual/collision/inertial frame mismatch after `+Z` `+90°` rotation** → Verify visually in `rk-viewport` and numerically with `com_fk_golden_test.rs` + CoM-vs-foot sole alignment checks.
- **Lower natural frequency ($\omega \approx 6.2\text{ rad/s}$ vs `7.7 rad/s`) increases shove capture distance** → Compensate via UVC step-timing adaptation and `waist_pitch/roll` balance authority tuned in `rk-mcp` sweeps.
- **Coupled double-joint limits in keyframes** → Assert both control-space (`θ_bend`) and URDF-space (`0.5×`) limits in `rk-model` unit tests.
