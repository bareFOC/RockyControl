# Tasks

> Delegable. Group 1 first; Group 2 after; Group 3 after; Group 4 integrates.

## 1. Shared torque-limited plant (OWNER: plant agent; FIRST)
- [x] 1.1 Add portable `JointPlant` to `firmware/rk-fw-core` using `EFFECTIVE_INERTIA`; refactor `BenchLoopbackBus` to use it; unit tests for saturation/velocity/torque modes. Verify `cargo test --manifest-path firmware/Cargo.toml -p rk-fw-core`.
- [x] 1.2 Refactor `rk-sim` `step_plant` to use `JointPlant` with same limits; keep fault knobs, world dynamics and multibus register I/O intact. Verify `cargo test -p rk-sim` (replay must stay exact).

## 2. Sensors + contacts (OWNER: sensor agent; depends on 1)
- [x] 2.1 Add seeded PRNG + sensor imperfection knobs (IMU noise/bias/latency, current noise, temp offset, encoder quantization) to `SimFaultInjection`; feed feedback. Verify repeatability test.
- [x] 2.2 Enforce unilateral GRF, friction/slip and CoP-in-polygon; add contact-disagreement fault. Verify contact tests.
- [x] 2.3 Verify shove produces a current spike detected by the kernel; add end-to-end impact test.

## 3. Full-app E2E (OWNER: ui agent; depends on 1/2)
- [x] 3.1 Migrate `crates/rk-ui/src/e2e_fall_recovery_test.rs` to `step_fixed`.
- [x] 3.2 Add walking displacement/contact alternation, combo stability and shove recovery tests through `RockyStudioApp` + `step_fixed`. Verify `cargo test -p rk-ui`.

## 4. Verification (OWNER: integrator; depends on all)
- [x] 4.1 Full host suite, firmware-core, thumb check, WASM check, lint, strict validate; update change status.
