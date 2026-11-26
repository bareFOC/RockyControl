# Proposal

## Why

`multi-robot-profiles` introduces the 28-DoF humanoid profile (`assets/robots/humanoid.urdf` + `humanoid.robot.toml`), coupled double-joints, toe push-off, 2-point heel+toe contact, and chest-IMU compensation, but the source URDF (`Kinematics/humanoid_description/humanoid.urdf`) was exported `-Y` forward instead of ROS REP-103 (`+X` forward, `+Y` left, `+Z` up), and all control gains, contact parameters, recovery keyframes, acceptance bounds, and `.copper` golden fixtures remain calibrated for the smaller Rocky Mk1 (`h_com ≈ 0.165 m` vs `h_com ≈ 0.25 m`). Without normalizing the URDF frame and tuning the 28-DoF dynamics in simulation, walking, shove recovery, and automated `sim-verify` gates cannot pass on the target humanoid.

## What Changes

- **Normalize URDF coordinate frame to ROS REP-103 (`+X` forward, `+Y` left, `+Z` up):** Rotate `assets/robots/humanoid.urdf` (and STL mesh/joint/origin transforms or root frame) from `-Y` forward to `+X` forward, `+Y` left, `+Z` up; verify joint axis sign directions (`l_knee_upper/lower`, `l_ankle_pitch`, `l_toe_pitch`, `waist_yaw/pitch/roll`, hips, arms); regenerate `crates/rk-robot-profile/src/generated/` via `xtask codegen-robots` and rebuild the embedded `rk-model` `.zip` bundle.
- **UVC / Balance / SLIP tuning for 28-DoF humanoid (`h_com ≈ 0.25 m`):** Run `rk-mcp` (`agentic-balance-tuning` experiment sweeps) to tune `BalanceGains` (`h_com_m`, `hip_height_m`, `foot_half_length_m`, `foot_margin_m`, UVC step timing and capture gains for $\omega = \sqrt{g / h_{\text{CoM}}} \approx 6.2\text{ rad/s}$) and `RunningConfig`.
- **Toe push-off & 2-point heel+toe contact tuning:** Verify and tune `toe_pitch` push-off and 2-point heel+toe contact in `rk-sim` across `walk_nominal` and `walk_shove` (including realistic sensor noise/latency).
- **28-DoF recovery keyframes & acceptance thresholds:** Verify and retune `default_recovery_library` and `default_combat_library` keyframes for 28-DoF joint limits and coupled double-joints, update `rk-mcp` / `rk-sim` acceptance bounds and `.copper` golden fixtures, and verify `cargo run -p xtask -- sim-verify`.

## Capabilities

### New Capabilities
<!-- None; all changes tune and validate existing capabilities for the 28-DoF humanoid. -->

### Modified Capabilities
- `bipedal-walking`: Tune `BalanceGains`, `RunningConfig`, capture-point timing ($\omega \approx 6.2\text{ rad/s}$ at `h_com ≈ 0.25 m`), and `toe_pitch` push-off for the 28-DoF humanoid.
- `simulator-fidelity`: Tune 2-point heel+toe contact dynamics and verify `walk_nominal` and `walk_shove` fidelity on the REP-103-normalized 28-DoF humanoid.
- `agentic-balance-tuning`: Update `rk-mcp` acceptance bounds and default gain/config targets for the 28-DoF humanoid geometry.
- `recovery-motion`: Retune `default_recovery_library` (and `default_combat_library`) keyframes to respect 28-DoF joint limits and coupled double-joints across all four fall orientations.
- `sim-verification-gate`: Regenerate committed `.copper` golden replay fixtures and experiment scripts for the tuned 28-DoF humanoid and pass `xtask sim-verify`.

## Impact

- Assets & codegen: `assets/robots/humanoid.urdf`, `assets/robots/humanoid.robot.toml`, `crates/rk-robot-profile/src/generated/*`, `crates/rk-model` embedded asset bundle.
- Control & simulation: `crates/rk-proto` / `crates/rk-control` (`BalanceGains`, `RunningConfig` defaults), `crates/rk-sim` (2-point heel+toe contact parameters, acceptance bounds), `crates/rk-model` (`default_recovery_library`, `default_combat_library`), `crates/rk-mcp`.
- Verification & fixtures: `tests/experiments/walk_nominal.{json,copper}`, `tests/experiments/walk_shove.json`, `tests/experiments/fall_recover.json`, `xtask sim-verify`.
