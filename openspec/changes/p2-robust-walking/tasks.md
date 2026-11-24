# Tasks

> Each group is subagent-delegable. Do not edit tasks.md checkboxes — parent handles that. No back-compat needed.

## 1. Phase clock (OWNER: uvc agent)
- [ ] 1.1 Add continuous phase accumulator to `UvcController` (`phi`, cadence-driven), phase-locked to `gait_phase_rad` on catch-step entry, continuous through ReadyWatch when walking. Verify `cargo test -p rk-control`.
- [ ] 1.2 Add phase continuity + support-alternation tests. Verify `cargo test -p rk-control`.

## 2. Capture foot placement (OWNER: uvc agent; depends on 1)
- [ ] 2.1 Replace swing modulation with Raibert/capture law using P1 `dcm_xy_m` + `com_vel`; normalized `vx` → `v_max≈0.5 m/s`; write only `dxis/dyis`; keep stance hip I-only. Verify `cargo test -p rk-control -p rk-sim`.
- [ ] 2.2 Add step-timing adaptation (shorten on capture breach, min 0.16 s). Verify tests.

## 3. Optional MPC (OWNER: control agent; parallel, gated)
- [ ] 3.1 DCM-form fixed-iteration f32 LIPM-MPC (N=6, T=0.08, 10–20 Hz, static buffers) behind a feature flag. Verify determinism + no regression; defer if risky and report.

## 4. Gait/estimator telemetry HUD (OWNER: UI agent; parallel to 2-3)
- [ ] 4.1 Add a HUD panel in `crates/rk-ui/src/control_screen.rs` surfacing the shared estimator/gait telemetry the controller actually uses: support-relative capture point (dcm_xy), CoM position/velocity, ZMP, gait phase, support foot, and step timing. Use `McuToHost.balance`/estimator telemetry fields; no new wire fields unless needed. Verify `cargo test -p rk-ui -p rk-viewport`.

## 5. Validation (OWNER: integrator; depends on all)
- [ ] 5.1 Shove/fall/replay suites pass; add walking-specific tests (speed tracking, disturbance recovery).
- [ ] 5.2 Full host suite + firmware-core tests + RP2350 thumb checks + lint + strict validate.
