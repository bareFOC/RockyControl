# Proposal

## Why

The simulator's walking tests pass while the walk they validate is wrong: measured with an instrumented 800-tick probe, the robot walks 48% faster than commanded, keeps a permanent 7° lean, spends 100% of ticks in `ActiveCatchStep`, has a negative capturability margin on 799/800 ticks, goes briefly airborne, reports half body weight in single support, and draws 1977 mA with realistic current noise (above the 1950 mA advisory). Root causes: a circular contact loop (controller derives contact from its own gait phase), `h_com_m = 0.25` vs a real sim CoM of ~0.72 m, `step_dur` computed but ignored while walking, a plant GRF bug, perfect-sensor defaults, and weak test assertions. This change makes simulated balance and walking genuinely good and measurably trustworthy.

## What Changes

- **P1 — Honest tests + plant fixes**: add measurable walk-quality metrics (speed, tilt, CoM height, slip, GRF, capturability, duty factor, current); fix per-foot GRF normalization (single support must be mg), accumulate foot slip, inject linear CoM impulse on shove, select stance by GRF; strengthen the rk-sim and UI walking tests with achievable quality assertions while leaving final thresholds for later phases.
- **P2 — Close the contact loop + model consistency**: add sensed contact/GRF to `RawServoFeedback` and use it in the estimator when valid (phase-derived fallback); plant publishes real contact into feedback and the kernel owns the telemetry mask; set default `h_com_m` to the FK-measured 0.72 m (clamp ≥0.80) with a golden test; retune the capturability trigger for continuous walking.
- **P3 — Timing + disturbances**: drive the walking gait accumulator from `step_dur` so Khadiv timing adaptation actually runs; add a walking-mode capture-breach test; add a realistic small-humanoid sensor profile and robustness scenarios (low friction, latency, stance stall, IMU dropout).
- **P4 — Quality tuning**: improve the swing-foot trajectory (toe-off/heel-strike dwell, bidirectional lateral placement) and sweep compliance/lag; tighten the quality assertions to final acceptance values and prove no current/fall faults.

## Capabilities

### New Capabilities
- None — this change improves the fidelity and validation of existing capabilities.

### Modified Capabilities
- `bipedal-walking`: step timing adaptation now applies during walking; walking quality has quantitative acceptance criteria.
- `state-estimation`: estimator consumes sensed contact/GRF when valid; CoM height matches the model.
- `simulator-fidelity`: plant GRF/slip/shove physics is corrected; realistic sensor profiles are available.
- `mcp-deterministic-stepping`: scripted experiments can include disturbance scenarios.

## Impact

Files: `crates/rk-control/src/{lib.rs,estimator.rs,uvc.rs,mass_table.rs}`, `crates/rk-proto/src/lib.rs`, `crates/rk-sim/src/lib.rs`, `crates/rk-ui/src/e2e_fixed_step_tests.rs`, `firmware/rk-fw-core/src/replay.rs` (event kind), `firmware/rk-fw-core/src/copper.rs` (if feedback fields change), UI/MCP gain bounds, relevant specs. No firmware board code, no control-kernel loop restructuring, no upstream bareFOC changes.

Risks: changing `RawServoFeedback` is a wire-format change (allowed; no back-compat required) and must keep replay bit-exact; changing `h_com_m` changes ω and therefore all capture-point math — must be retuned together with the margin trigger; tightening tests may expose further plant deficiencies, which must be fixed rather than suppressed.

Deferred: full LIPM-MPC / whole-body control, learned policies, uneven-ground models, hardware bring-up (separate scope).
