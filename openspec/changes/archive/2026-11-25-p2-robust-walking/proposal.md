# Proposal

## Why

With the FK-based CoM estimator in place (P1), the robot now has a physically correct support-relative capture point, but the gait is still a timer-driven synthetic phase with no Raibert foot placement, no step-timing adaptation, and no preview. Robust walking requires a phase-locked gait clock and capture-based foot placement that reacts to disturbances every tick.

## What Changes

- Add a phase accumulator (`phi += 2π·f·dt`, L/R offset π) in `UvcController`, phase-locked to the existing `gait_phase_rad` on `ActiveCatchStep` entry; run continuously through `ReadyWatch` when walking, driving support swaps instead of fixed timers.
- Replace the simulator-only `DcmSwingHint` swing modulation with a **Raibert/capture foot-placement law** using the P1 support-relative DCM: `x_f = ξ_rel + K_v(v_des − v)/ω`, with normalized `vx` mapped to a conservative `v_max ≈ 0.5 m/s`. Write landing targets **only** to `dxis/dyis` (swing foot) — never stance `dxi/dyi`.
- Add Khadiv-style step-timing adaptation: shorten step time when capture error is large, clamped to ≥0.16 s.
- Optional (later in stage): small fixed-iteration f32 DCM-form LIPM-MPC (N=6, T=0.08 s, 10–20 Hz) for multi-step preview; deterministic, static buffers.
- Keep all math fixed-dt f32 inside `DirectServoKernel::step`; replay remains bit-exact.

## Capabilities

### New Capabilities
- `bipedal-walking`: Reactive, capture-based bipedal gait with a phase-locked clock, Raibert foot placement, and step-timing adaptation usable identically in simulation and firmware.

### Modified Capabilities
- `state-estimation`: P2 consumes the support-relative DCM; no estimator behavior change.
- `unified-control-application`: new deterministic gait state lives in the shared kernel.

## Impact
- `rk-control` (`uvc.rs`, `estimator.rs` consumer, `lib.rs`), `rk-proto` (optional WalkCmd/gain additions), `rk-ui`/`rk-viewport` (gait telemetry), `rk-sim` (walking tests).
- Risk: ω/gait retuning after P1; validate shove/fall/replay suites.
