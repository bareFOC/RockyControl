# P1 Estimator Upgrade — Implementation Research (summary)

Full research performed 2026-11-24; this file captures the key findings referenced by the `p1-mass-weighted-com-estimator` OpenSpec change.

## Baseline
- `crates/rk-control/src/estimator.rs:106-134`: CoM from IMU-angle proxy (`com_x/z = lever*sin(roll/pitch) + arm offsets`), hardcoded hip height from `LegGeometry::rocky_mk1().nominal_height = 0.62`.
- `estimator.rs:136-178`: velocity = finite differences + `0.55·FD + 0.45·(r·ω_imu)` gyro blend, EMA α=0.30, air-gated.
- `estimator.rs:180-186`: ω from dynamic proxy height (~0.57 m → 4.15 rad/s); DCM in pelvis frame, margin foot-fixed ±2.8 cm.
- `lib.rs:607-615`: `RawServoFeedback.velocity_rad_s` already converted to `measured_vel_rad_s` but NOT passed to estimator.
- `uvc.rs:441-448`: bias-compensated `pitch_eff/roll_eff` computed internally; estimator uses raw IMU tilt.

## Hard blocker: URDF has no mass properties
`assets/sample_humanoid/rocky_mk1.urdf` has only `<visual>` tags — no `<inertial>`, `<mass>`, or `<inertia>` anywhere. Resolved by user decision: author real BOM masses in URDF as single source of truth; generate checked-in constant table.

## Algorithms

**Mass-weighted FK CoM:** for links in parent-first order with mass m_i, local COM c_i, world transform T_i: `M = Σm_i`, `p_com = (1/M) Σ m_i (T_i · c_i)`. O(n), fixed order, f32, `inv_mass` constant.

**CoM Jacobian × q̇:** `v_com = (1/M) Σ_i m_i Σ_{j≼i} z_j × (p_i − p_j) · q̇_j`. O(n) two-sweep recursion using measured joint velocities; no finite differences. `k`/nalgebra NOT on MCU (allocation); host uses `k::Chain` only for golden values.

**Support-relative DCM:** `ξ_rel = (p_com − p_foot) + v_com/ω` with v_foot≈0; root FK at stance foot (double-support midpoint). Margin ±2.8 cm on ξ_rel. Reset position history on support swap. `h_com_m` fixed (0.25 m → ω≈6.26) for DCM + ZMP; independent of UVC hip_height_m (0.62).

**Bias:** 1-tick-lagged `pitch_bias()/roll_bias()` from UVC passed to estimator; no state-machine refactor. FK CoM removes IMU dependence for position anyway.

## Settled decisions
Real BOM masses; keep IMU-required validity contract; h_com=0.25; 1-tick bias lag; FK scoped to 21-DoF with proxy fallback; double-support midpoint.

## Validation
Host golden CoM tests from `k::Chain`; Jacobian test; support-swap spike test (<0.5 m/s); rewritten DCM test with joint-velocity impulse; replay bit-exactness with nonzero velocities; full shove/fall suites.

## Task groups (see OpenSpec tasks.md)
A: mass properties/URDF · B: com_fk + estimator rewrite · C: wiring/protocol (h_com, PROTOCOL_VERSION=3, bias accessor) · D: UI/host surfaces · E: validation/docs.
