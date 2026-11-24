# Tasks

> Each group is subagent-delegable with clear ownership. Do not edit tasks.md checkboxes — parent handles that.

## 1. Embassy WASM harness proof (OWNER: runtime agent; FIRST)
- [x] 1.1 Add a minimal harness crate/module with `embassy-time/mock-driver` + `generic-queue-8`, raw executor + no-op pender, `step()` that advances 5 ms and polls to quiescence. Add a native test asserting exactly N control ticks after N advances (Ticker task counter). Verify `cargo test -p <harness>` and `cargo check --target wasm32-unknown-unknown -p <harness>`.
- [x] 1.2 Verify feature unification: RP2350 thumb checks still pass with harness in workspace (no tick-hz conflict). Report `cargo tree -e features` evidence.

## 2. Shared control task + IO trait (OWNER: runtime agent; depends on 1)
- [x] 2.1 In `rk-fw-core`, define `RobotIo` trait and `#[embassy_executor::task] control_task` wrapping `ControlTask::step`, with a 5 ms Ticker. Keep existing control logic intact. Verify host + firmware-core tests.
- [x] 2.2 Port `rk-sim::SimulatedServoPlant` to implement `RobotIo`; port `CopperSimRunner` to run through the harness task (keep COBS exchange and replay recording). Verify `cargo test -p rk-sim -p rk-fw-core`.
- [x] 2.3 Update `rk-rp2350` hardware backend to implement/spawn the shared task under its `#[embassy_executor::main]`; keep existing feature gates. Verify thumb checks.

## 3. Fixed-step app integration (OWNER: app agent; depends on 2)
- [x] 3.1 Replace one-tick-per-frame step in `rk-ui/src/lib.rs` with a bounded fixed-step accumulator driving harness.step(); dt is a fixed 5 ms constant, catch-up capped. Verify UI tests.
- [x] 3.2 Report achieved control tick rate (not FPS) in `rk-link::SimTransport` and HUD; add indicator when frames cannot sustain 200 Hz. Verify `cargo test -p rk-link -p rk-ui`.
- [x] 3.3 Wire MCP/headless loops to the same scheduler — DEFERRED/documented: headless MCP keeps variable-dt `step_exchange(0.01)` as a separate timing domain (rustdoc on `RobotTransport::step_exchange`); fixed-step is interactive-app only.

## 4. Plant physics + world state (OWNER: physics agent; parallel to 3, depends on 2)
- [x] 4.1 Add world CoM x/z position+velocity to `SimulatedServoPlant`, computed via shared `com_fk`/leg geometry from joint angles. Verify plant tests.
- [x] 4.2 Add per-foot contact flags/points, GRF, penetration/slip; generalize running spring to walking stance. Replace synthetic sway with contact/CoP moments. Verify tests.
- [x] 4.3 Add end-to-end walking test: `walk_enabled=true, vx>0` for seconds → forward displacement, alternating contacts, bounded tilt. Verify `cargo test -p rk-sim`.

## 5. Viewport + HUD translation (OWNER: UI agent; depends on 4)
- [x] 5.1 Add world translation to telemetry (derive from plant FK; add wire field or reuse), apply root translation in `rk-viewport` scene. Verify viewport tests.
- [x] 5.2 Show achieved velocity/displacement/contact and achieved control rate in gait HUD. Verify UI tests.

## 6. WASM replay export + final verification (OWNER: integrator; depends on all)
- [x] 6.1 Export `.copper` replay via `PlatformSaver` in WASM from config screen; make recording limit configurable/surfaced. Verify WASM build + UI tests.
- [x] 6.2 Full verification: host suite (163 tests), firmware-core (25), RP2350 thumb feature matrix (5 configs), WASM `cargo check --target wasm32-unknown-unknown`, lint, strict validate.
- [x] 6.3 Update `reports/ROCKY_ALGORITHM_ASSESSMENT.md` with new simulation-first status; commit/sync/archive/push.
