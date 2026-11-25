# Proposal

## Why

After the fidelity-hardening fixes, the simulator still has ideal sensors, a first-order heuristic actuator, scripted contacts that bypass the controller's contact estimator, and no full-app end-to-end tests on the interactive `step_fixed` path. Gains (UVC/DCM/compliance/impact) tuned against perfect sensors and a scripted plant are provisional. This change makes the plant and sensors credible enough for tuning work, and makes "I tuned it in the app" reproducible.

## What Changes

- **Seeded sensor imperfection**: add a deterministic PRNG to `SimFaultInjection` driving IMU gyro/accel noise + bias + latency, current-sense noise, temperature offset and encoder quantization. No wall clock, no `rand`; same seed → same samples.
- **Torque-based actuator dynamics**: compute `τ = kp·err − kd·ω + gravity_torque`, clamp to calibrated current/torque limits, integrate `ω += τ/I·dt` using `rk_control::impact::EFFECTIVE_INERTIA`, derive reported current from `|τ|`; add friction/bandwidth knobs. Unify with `BenchLoopbackBus` so firmware bench loopback and simulator share one plant model.
- **Impact path validation**: make an applied shove produce a joint current spike large enough for the kernel's impact detector (`rk-control/src/impact.rs`) to flag; add a test proving the full path.
- **Contact refinement**: enforce unilateral contacts, a friction/slip threshold and CoP-inside-support-polygon in `step_world_dynamics`; add a fault that desynchronizes plant contacts from the controller's phase-scripted `contact_mask` so estimator robustness can be tested.
- **Full-app `step_fixed` end-to-end tests**: walking displacement + contact alternation through `RockyStudioApp`, combo/strike playback staying upright, shove recovery, and migrate the fall/get-up test from `step_exchange` to `step_fixed`.

## Capabilities

### New Capabilities
- `simulator-dynamics`: deterministic imperfect sensors, torque-limited actuator dynamics and unilateral contacts shared with the firmware bench loopback.

### Modified Capabilities
- `simulator-fidelity`: adds physics/sensor credibility and full-app test coverage on the fixed-step path.
- `combat-behaviors`: impact detection is validated end-to-end through the simulator's shove → current path.

## Impact

Files: `crates/rk-sim/src/lib.rs`, `firmware/rk-fw-core/src/servo_bus.rs`, `crates/rk-ui/src/e2e_fall_recovery_test.rs`, new E2E test modules, relevant specs. No upstream bareFOC changes; no control-kernel changes. Determinism and `f32` fixed-dt parity are mandatory.

Deferred (large tier): time-origin normalization for portable cross-path replay, deterministic scripted experiment runner, targeted multi-body effects beyond support-polygon CoP/swing-foot coupling, and HIL validation.
