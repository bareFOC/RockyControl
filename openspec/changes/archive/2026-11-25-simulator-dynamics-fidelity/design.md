# Design

## Context

The simulator currently uses first-order position tracking with synthesized current (`rk-sim/src/lib.rs` step_plant), ideal sensors, and contacts derived from the controller's reported foot offsets. The firmware bench loopback (`firmware/rk-fw-core/src/servo_bus.rs` `BenchLoopbackBus`) uses a different first-order constant (18 vs ~7–21 rad/s), so tuning does not transfer. `rk_control::impact::EFFECTIVE_INERTIA` already provides URDF-derived per-link inertia for the impact detector.

## Goals / Non-Goals

**Goals:** deterministic imperfect sensors; torque-limited actuator dynamics shared with bench loopback; constraint-aware contacts; end-to-end validation through `step_fixed`.

**Non-Goals:** a general rigid-body physics engine; wall-clock randomness; changing the control kernel; hardware/HIL work; time-origin normalization (large tier).

## Decisions

1. **PRNG**: fixed-size xorshift/LCG state in `SimFaultInjection`, seeded from a `u64` config; all noise draws go through one `next_f32()` helper. No `rand` dependency, no wall clock. Seed included in fault-injection serde so experiments are reproducible.
2. **Actuator**: build a shared `JointPlant` model in `rk-fw-core` (portable `no_std`, `f32`) used by both `BenchLoopbackBus` and `rk-sim`. State: angle, velocity, temperature. `τ = kp·(target−angle) − kd·velocity + gravity_torque`, clamp to `±MAX_TORQUE_NM`, `ω += τ/I·dt` using `EFFECTIVE_INERTIA`, `α = ω` integrated into angle; current derived from `|τ| / nm_per_amp`. Velocity/torque modes share the same dynamics. Existing lag/fault knobs map to bandwidth/friction multipliers.
3. **Impact**: shove impulses apply to joint velocity/current through the plant so the current spike reaches the kernel's detector naturally; remove the direct torso-rate-only shortcut or keep it only as an additional effect. Verify with a full-app test.
4. **Contacts**: in `step_world_dynamics`, reject negative GRF, apply slip when tangential displacement exceeds `μ·grf`, and compute CoP from contact points clamped inside their support polygon; add a fault toggling plant-vs-controller contact disagreement.
5. **E2E tests**: parameterize the existing fall/get-up harness to use `step_fixed`; add walking-displacement, combo-stability and shove-recovery tests on `RockyStudioApp`. Keep all tests deterministic from a clean reset.

## Risks / Trade-offs

- Replacing the first-order plant changes recorded replay logs; existing bit-exact replay tests assert self-consistency (record then verify), so they should still pass, but numeric thresholds in plant tests may need retuning.
- Sharing the plant with `rk-fw-core` adds a `no_std` module there; keep it hardware-agnostic.
- Sensor noise makes HUD plots noisy; keep noise amplitudes configurable and small by default.
- Friction/slip and CoP clamping add nonlinearity; keep fixed-dt `f32` and avoid branch-dependent nondeterminism.
