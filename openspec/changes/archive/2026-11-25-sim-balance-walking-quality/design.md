# Design

## Context

An instrumented 800-tick probe of the passing walk test measured: 0.371 m/s vs 0.25 commanded, final pitch −0.1245 rad, `ActiveCatchStep` 800/800 ticks, negative foot margin 799/800 ticks, 21 flight ticks, single-support GRF ~7.4 N vs 14.6 N body weight, and 1977 mA with 3% current noise. Root causes: circular phase-derived contact, `h_com_m=0.25` vs FK ~0.72 m, `step_dur` dead code while walking, per-foot `+0.5·mg` GRF bug, zero-noise defaults, weak assertions.

## Goals / Non-Goals

**Goals:** make the simulator demonstrate measurably good balance and walking; close the contact loop; make ω model-consistent; make step timing adapt during walking; add realistic-noise robustness; tighten acceptance metrics.

**Non-Goals:** hardware changes; full LIPM-MPC/WBC; learned policies; uneven-ground models; changing the deterministic 200 Hz executor or replay verification semantics.

## Decisions (by phase)

**P1 — Honest tests + plant fixes.** Add `FootContact::slip_distance_m` and accumulate per step; normalize GRF by loaded-foot count; make `apply_push_impulse` inject a linear world CoM impulse and choose stance by GRF; add a `WalkQuality` measurement struct in rk-sim (mean speed, peak/steady tilt, CoM height range, lateral drift, slip, GRF stats, duty factor, current peaks, margin fraction); expose it from the runner for tests. Strengthen the sim and UI walking tests with assertions achievable after P1 fixes (GRF≈mg, no flight, current headroom, slip, measured speed/tilt/margin reported and loosely bounded). Each phase ends with all tests green; final thresholds land in P4.

**P2 — Contact loop + model.** Extend `RawServoFeedback` with `contact_mask`, `grf_n: [f32;2]`, and a validity bit (`estimator_valid::CONTACT_SENSED` or reuse GRF). Kernel passes sensed contact to `KinematicEstimator::step` when valid, phase mask otherwise; sim plant publishes its real contact into feedback after the world step and stops overwriting the kernel mask in telemetry. Change default `h_com_m` to 0.72 (kernel clamp max ≥0.80), update UI/MCP bounds and stale call sites, add an FK golden test. Retune capturability trigger: no margin-based step trigger during steady walking; use support-polygon breach with swing-phase gating.

**P3 — Timing + disturbances.** Walking phase accumulator advances by `π/step_dur · dt` (clamped to 0.16..0.45 s nominal bounds); update gait clock tests and add `walking_capture_breach_shortens_step`. Add `SimFaultInjection::realistic_small_humanoid()` (gyro 0.02 rad/s, accel 0.04 m/s², biases 0.01, latency 2 ticks, current noise 3%, encoder 0.002 rad) and scripted scenarios for low friction, latency, stance stall and IMU dropout; add robustness variants of walking/shove tests.

**P4 — Quality tuning.** Improve swing-foot trajectory (toe-off/heel-strike dwell, bidirectional lateral placement by removing `max(0.0)` clamps where safe) and sweep `servo_lag_factor`/compliance; tighten all P1 quality assertions to final acceptance values; prove no over-current/fall faults under the realistic profile.

## Risks / Trade-offs

- `RawServoFeedback` field additions change the wire format; replay must stay bit-exact (tests will verify).
- `h_com_m` changes ω globally; capturability retuning must be simultaneous or walking degrades.
- Tightening tests can expose further plant deficiencies; fix them rather than weaken assertions.
- Each phase must leave the full workspace green before the next begins; subagents own disjoint files per phase.
