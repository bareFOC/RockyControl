# Design

## Context

See `proposal.md` and `reports/P1_ESTIMATOR_RESEARCH.md` (the detailed research backing this change). Current estimator: CoM from IMU-angle proxy (`estimator.rs:106-134`), velocity from finite differences + gyro blend (`:136-178`), dynamic ω from proxy height (`:182`). Joint velocity is already available (`RawServoFeedback.velocity_rad_s`, plumbed at `lib.rs:607-615`) but unused by the estimator. The bundled URDF has NO `<inertial>` data — real BOM masses must be authored as the single source of truth.

## Goals / Non-Goals

**Goals:** physically correct CoM/velocity from joint state + mass table; support-relative DCM with fixed configurable h_com; deterministic and bit-replayable; single shared code path host/MCU.

**Non-Goals:** changing UVC's control partition or state machine (only a bias accessor + 1-tick-lagged use); adding torque/impedance (v2); walking/running/combat behaviors (P2–P4); changing the URDF's visual/geometric data.

## Decisions (settled)

1. **Mass properties:** author `<inertial>` tags from the real BOM (~1–2 kg total, 45 cm robot) in the URDF; generate a checked-in `mass_table.rs` constant table (22 links, parent-first). The URDF is authoritative; MCU never parses XML. Host test asserts table matches URDF.
2. **CoM math:** new pure `com_fk.rs` in `rk-control`: `compute_com(angles)` and `com_velocity(angles, velocities)` via O(n) mass-weighted Jacobian recursion. No `k`/nalgebra on MCU (allocation); host uses `k::Chain` only for golden test values.
3. **Support-relative DCM:** root FK at stance foot (double-support midpoint); `ξ_rel = r_com/foot + v_com/ω`. Margin checks/mapping remain swing-foot-only (`DcmSwingHint`); reset position history on support swap to avoid velocity spike.
4. **h_com:** add `BalanceGains.h_com_m` default 0.25 m (ω≈6.26), clamp [0.12, 0.60], bump `PROTOCOL_VERSION` to 3. UVC `hip_height_m` (0.62 m IK geometry) stays independent — document ratio ~0.4.
5. **Bias:** expose `UvcController::bias_compensated_attitude()`; kernel passes previous-tick bias (`pitch_bias()/roll_bias()`) into estimator (1-tick lag, no state-machine refactor). With FK CoM, IMU tilt no longer affects position anyway.
6. **Validity:** keep COM/DCM validity gated on `imu.available` (no new validity bit).
7. **Scope:** FK path for 21-DoF Rocky; proxy fallback for other joint counts (guard `joint_count == 21`).

## Risks / Trade-offs

- [ω increases ~50% vs proxy] → `dcm_step_gain` may need retuning; validate shove/fall suites before P2.
- [Mass table accuracy] → golden tests only validate math against the table, not physics; real calibration/measurement is future work. Document masses as authored BOM estimates.
- [Replay determinism] → fixed traversal order, no hash maps, f32 only; replay test with nonzero velocities is mandatory.
- [Protocol bump] → all host/MCU crates rebuilt in lockstep (no back-compat per AGENTS.md).

## Verification

- Golden CoM values for 3–4 poses from host `k::Chain` compared to `com_fk`;
- Jacobian test (joint velocity → CoM velocity), support-swap spike test;
- Rewritten DCM test with joint-velocity impulse;
- Existing shove/fall/replay suites must pass; full host suite + firmware-core tests + RP2350 thumb feature checks.
