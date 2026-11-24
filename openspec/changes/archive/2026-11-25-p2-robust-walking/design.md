# Design

## Context

P1 delivers a physically correct support-relative capture point (`EstimatorTelemetry` + `DcmSwingHint`) in the shared kernel. Current `UvcController` (rk-control/uvc.rs) uses a synthetic phase that only advances during `ActiveCatchStep` and a timer-driven step duration (0.32 s at default cadence). This stage adds reactive walking on top of the P1 estimator.

## Goals / Non-Goals

**Goals:** continuous phase clock, capture/Raibert swing-foot placement, step timing adaptation, deterministic replay, optional small MPC. **Non-Goals:** torque-control impedance (v2), running (P3), combat (P4), changing the P1 estimator.

## Decisions

1. **Phase accumulator** in `UvcController`: `phi += 2π·cadence·dt` with L/R π offset; phase-lock to `gait_phase_rad` on catch-step entry; continuous through `ReadyWatch` when `walk_enabled`.
2. **Foot placement** using P1 `dcm_xy_m` (support-relative) and `com_vel`: `x_f = ξ_rel + K_v(v_des − v)/ω`; normalized WalkCmd.vx → v_des with conservative `v_max≈0.5 m/s` (≤ `max_dx=0.16 m` at T=0.32 s). Write only `dxis/dyis` (swing foot); stance `dxi/dyi` stays UVC I-only.
3. **Step timing**: shorten step when `|ξ_rel| > foot_margin`, `T ∈ [0.16, 0.45] s`.
4. **Optional MPC**: DCM-form QP (N=6, T=0.08 s) with fixed f32 buffers, updated 10–20 Hz, tracked by the 200 Hz law. Defer if it risks determinism.
5. **Determinism**: all state in `UvcController`/kernel; fixed dt from caller; `f32` + libm; replay unchanged.

## Risks / Trade-offs

- [ω/gait retuning] → P1 fixed h_com=0.25 → ω≈6.26; validate shove tests.
- [Phase-lock discontinuity] → test phase continuity at catch-step entry.
- [MPC complexity] → optional; defer if it destabilizes replay.
