# Tasks

> Sequential phases. Each phase: delegate → verify full workspace green → check off → next phase. Final: archive + sync + commit + push.

## P1 — Honest tests + plant fixes (OWNER: sim agent)
- [x] 1.1 Add `FootContact::slip_distance_m`, accumulate slip; normalize per-foot GRF by loaded-foot count.
- [x] 1.2 Shove: inject linear world CoM impulse; select stance leg from GRF, not `last_balance.support_foot`.
- [x] 1.3 Add `WalkQuality` measurement struct + expose from `CopperSimRunner`; strengthen rk-sim + rk-ui walking tests with P1-achievable quality assertions (GRF≈mg, no flight, current headroom, slip; report speed/tilt/margin). Verify `cargo test --workspace`.

## P2 — Contact loop + model (OWNER: control agent; after P1)
- [x] 2.1 Add sensed contact/GRF to `RawServoFeedback`; estimator prefers it when valid, phase fallback otherwise; sim publishes real contact into feedback and stops overwriting kernel mask.
- [x] 2.2 Default `h_com_m` → 0.72, clamp ≥0.80, update UI/MCP bounds + call sites; FK golden test; retune capturability trigger for continuous walking.
- [x] 2.3 Verify replay bit-exact, all tests green.

## P3 — Timing + disturbances (OWNER: gait agent; after P2)
- [x] 3.1 Drive walking phase accumulator from `step_dur`; update gait tests; add `walking_capture_breach_shortens_step`.
- [x] 3.2 Add realistic sensor profile + scripted disturbance scenarios; robustness variants of walking/shove tests. Verify all green.

## P4 — Quality tuning + finish (OWNER: tuning agent; after P3)
- [x] 4.1 Swing trajectory (toe-off/heel dwell, lateral placement) + compliance/lag sweep.
- [x] 4.2 Tighten quality assertions to final acceptance values; prove no over-current/fall under realistic profile. Verify full gate.
- [x] 4.3 Archive + sync OpenSpec change; commit and push.
