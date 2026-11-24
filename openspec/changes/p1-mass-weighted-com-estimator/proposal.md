# Proposal

## Why

The current kinematic estimator (`rk-control/src/estimator.rs`) computes CoM from an IMU-angle proxy and derives velocity via finite differences plus a hand-tuned gyro blend. That proxy ignores leg joint positions and measured joint velocity, uses a dynamic CoM height (ω≈4.15 rad/s) mismatched with the foot-fixed ±2.8 cm capturability margin, and never sees UVC's bias-compensated attitude. Now that joint velocity feedback is plumbed into `RawServoFeedback`, we can replace it with a physically correct mass-weighted FK CoM and analytic `J_com·q̇`, the foundation all later locomotion stages (walking, running, combat) depend on.

## What Changes

- **BREAKING wire:** Add `BalanceGains.h_com_m` (default 0.25 m) and bump `PROTOCOL_VERSION` 2 → 3. All host/MCU builds updated in lockstep.
- Author real per-link mass properties (`<inertial>` tags) in `assets/sample_humanoid/rocky_mk1.urdf` as the single source of truth, and check in a generated constant table `rk-control/src/mass_table.rs` (22 links, parent-first order).
- Add a pure `rk-control/src/com_fk.rs` module: `compute_com(angles)`, `com_velocity(angles, velocities)` using mass-weighted FK and an O(n) `J_com·q̇` recursion. No allocation, no `k` crate on MCU (host uses `k::Chain` only to generate golden test values).
- Rewrite the estimator to use FK CoM and analytic velocity; remove the IMU-proxy constants and FD+gyro blend. Root FK at the stance foot (or double-support midpoint) for **support-relative DCM** `ξ_rel = r_com/foot + v_com/ω`; keep the swing-foot-only DOF partition.
- Use a fixed configurable `h_com_m` for ω and ZMP (ω≈6.26 rad/s at 0.25 m). Keep UVC's `hip_height_m` independent.
- Share bias-compensated attitude from UVC with the estimator via a 1-tick-lagged bias accessor (no UVC state-machine refactor). Keep `valid_mask` requiring IMU availability (unchanged contract).
- Scope FK to the 21-DoF Rocky topology; keep the existing proxy path as fallback for other joint counts.
- Migrate shove tests to inject real joint-velocity impulses; add golden CoM tests, Jacobian tests, support-swap spike test, and replay-determinism fixture.

## Capabilities

### New Capabilities
- `state-estimation`: Provides a physically correct, deterministic CoM/CoM-velocity estimate and support-relative capture point from joint encoders, joint velocities, and mass properties, usable identically on host and MCU.

### Modified Capabilities
- `unified-control-application`: The estimator upgrade changes the deterministic data the shared control application computes; replay/determinism requirements now cover FK-based estimation.

## Impact

- **URDF/assets:** `assets/sample_humanoid/rocky_mk1.urdf` gains `<inertial>` blocks.
- **Crates:** `rk-control` (new `mass_table.rs`, `com_fk.rs`, rewritten estimator), `rk-proto` (BalanceGains field, protocol version), `rk-model` (host golden tests), `rk-ui` (h_com slider, relabel DCM telemetry), `rk-sim`/`rk-fw-core` (tests, replay fixture).
- **Risk:** ω increases ~50% vs current proxy → capturability sensitivity changes and `dcm_step_gain` may need retuning; validate with shove/fall suites before proceeding to P2.
