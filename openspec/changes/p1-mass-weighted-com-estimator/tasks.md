# Tasks

> Delegation note: each numbered group is a self-contained unit with clear file ownership, intended for one subagent. Do not edit tasks.md checkboxes — parent handles that. No backward compatibility needed.

## 1. Mass properties & URDF (OWNER: model/mass agent)

- [x] 1.1 Add `<inertial>` blocks (`<mass>`, `<origin xyz>`, `<inertia>`) to all 22 links of `assets/sample_humanoid/rocky_mk1.urdf` using realistic BOM masses for a ~1–2 kg, 45 cm humanoid; document totals in a comment. Verify URDF still parses in rk-model.
- [x] 1.2 Add checked-in `crates/rk-control/src/mass_table.rs` generated from those masses: `TOTAL_MASS_KG`, `INV_TOTAL_MASS`, `LinkMass { parent, com:[f32;3], mass, ... }`, `LINKS: [LinkMass; 22]` in fixed parent-first order; export from `lib.rs`. Verify `cargo test -p rk-control`.
- [x] 1.3 Add host test in `crates/rk-model` asserting the checked-in mass table matches values parsed from the URDF's `<inertial>` tags. Verify `cargo test -p rk-model`.

## 2. CoM FK + analytic velocity + estimator rewrite (OWNER: control kernel agent; depends on Group 1)

- [x] 2.1 Add pure `crates/rk-control/src/com_fk.rs`: `compute_com(angles: &[f32; MAX_JOINTS]) -> [f32;3]`, `com_velocity(angles, velocities) -> [f32;3]` via O(n) mass-weighted Jacobian recursion; no allocation, `libm`/deterministic trig only. Add unit tests including golden values from host `k::Chain` (3–4 poses) and a joint-velocity→CoM-velocity test. Verify `cargo test -p rk-control`.
- [x] 2.2 Rewrite `KinematicEstimator::step` in `crates/rk-control/src/estimator.rs` to use `compute_com` + `com_velocity`; add `measured_vel_rad_s: &[f32; MAX_JOINTS]` parameter; delete IMU-proxy constants, FD+gyro blend. Keep EMA/air-gating only as optional smoothing on analytic velocity. Verify `cargo test -p rk-control`.
- [x] 2.3 Support-relative DCM: root FK at stance foot (double-support midpoint from contact_mask); `ξ_rel = r_com/foot + v_com/ω`; fixed `h_com_m` for ω and ZMP; update margin/swing hint (swing-foot only) and reset position history on support swap. Add support-swap spike test (<0.5 m/s). Verify `cargo test -p rk-control`.
- [x] 2.4 Scope FK to `joint_count == 21`; keep current proxy path as fallback for other counts. Update estimator module docs. Verify `cargo test -p rk-control`.

## 3. Wiring & protocol (OWNER: kernel/proto agent; depends on Group 2)

- [x] 3.1 `rk-proto`: add `BalanceGains.h_com_m` (default 0.25), finite check, clamp [0.12,0.60] in kernel config handling (`rk-control/src/lib.rs`), bump `PROTOCOL_VERSION` to 3. Verify round-trip + config tests.
- [x] 3.2 `rk-control/src/lib.rs`: pass `&measured_vel_rad_s` and bias-compensated attitude into `self.estimator.step()` (`:481-491`); add `UvcController::bias_compensated_attitude()` / `pitch_bias()`/`roll_bias()` accessors in `uvc.rs`, use previous-tick bias (1-tick lag). Keep validity gating on IMU availability. Verify `cargo test -p rk-control -p rk-proto`.
- [x] 3.3 Add `h_com_m` slider in `crates/rk-ui` balance-gain panel; relabel DCM telemetry as support-relative. Verify `cargo test -p rk-ui -p rk-viewport`.

## 4. Validation & docs (OWNER: integrator; depends on all groups)

- [x] 4.1 Rewrite `rk-control/src/lib.rs` DCM test (`kinematic_dcm_triggers_step_outside_2_8cm_margin`) to inject a real joint-velocity impulse; ensure shove/fall/replay suites still pass. Verify `cargo test -p rk-control -p rk-sim`.
- [x] 4.2 Add replay determinism fixture with nonzero joint velocities; verify bit-exact estimator telemetry across replay.
- [x] 4.3 Full verification per AGENTS.md: host suite (`cargo test -p rk-proto -p rk-control -p rk-sim -p rk-model -p rk-link -p rk-viewport -p rk-ui -p rk-mcp -p rk-fw-core`), firmware-core tests, RP2350 thumb feature checks, `sh scripts/lint.sh`, `openspec validate p1-mass-weighted-com-estimator --strict`.
- [x] 4.4 Update `reports/ROCKY_ALGORITHM_ASSESSMENT.md` P1 notes if needed; report final status.
