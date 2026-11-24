# Design

## Context

A WASM spike compiled successfully: `embassy-time = { features = ["mock-driver", "generic-queue-8"] }`, `critical-section/std`, `embassy-executor` with no platform feature, target `wasm32-unknown-unknown`. Research showed current Embassy (0.10/0.5.1) has no `Executor::try_tick()` on high-level executors; use `embassy_executor::raw::Executor::poll()` with a no-op pender and a manual loop: `MockDriver::advance(5ms)` → poll to quiescence. The physics audit found the plant has no world state or contacts and the UI steps once per frame, so Embassy-WASM alone is insufficient — both the runtime boundary and the plant must change.

## Goals / Non-Goals

**Goals:** one Embassy control task in browser/native/MCU; deterministic 200 Hz simulated time independent of FPS; a physics plant with world state, contacts and GRF; visible forward walking; end-to-end walking test; bit-exact replay preserved.

**Non-Goals:** changing control algorithms (P1–P4 stay); ESP32-P4 bring-up; full rigid-body dynamics; vision; learning.

## Decisions

1. **Shared control task + IO trait:** `rk-fw-core` exposes `trait RobotIo` (sync `read_feedback`/`write_command`/host command/telemetry, matching current path) and `#[embassy_executor::task] async fn control_task(io: RobotIoImpl)` with one `Ticker::every(5ms)`. Keep the existing `ControlTask::step()` as the pure kernel call inside it.
2. **Harness:** new `rk-embassy-harness` crate (or module in rk-fw-core behind `std`/wasm feature) with `mock-driver` + `generic-queue-8`, raw executor + `pender_impl!` no-op, `Box::leak`ed executor, `step()` = advance 5 ms + poll until no progress. Called from eframe `update()` with a bounded accumulator (max ~4 steps/frame); on native tests `step_n(n)`.
3. **Feature hygiene:** mock-driver forces 1 MHz `tick-hz`; keep it in the harness crate only (host/WASM builds); MCU builds keep their own time driver. Verify feature unification with `cargo tree -e features`.
4. **Determinism:** no wall-clock in the control path; task wake order not relied upon (single control task in harness); all plant/controller math f32 fixed-dt; replay records per-tick dt and still verifies `ControlTask`.
5. **Plant physics:** extend `SimulatedServoPlant` with world CoM x/z + velocity, per-foot contact points/flags, GRF, penetration/slip. Use `rk-control::com_fk` + leg geometry for foot/CoM FK. Replace synthetic sway with support-polygon/CoP moments; keep 2-axis tilt integrator. Generalize running spring to per-foot stance contacts.
6. **Viewport/UI:** add world translation to telemetry (derive from plant FK) and apply root translation in the viewport scene; show achieved speed/displacement/contact and achieved control rate in HUD.
7. **Transport:** `rk-link::SimTransport` delegates to the harness `step()` instead of `CopperSimRunner::tick` (but keeps COBS exchange and faults); `packet_rate_hz` becomes achieved control ticks/s.
8. **WASM replay export:** use existing `PlatformSaver` to download `.copper` JSON in browser.

## Risks / Trade-offs

- [WASM+mock untested by Embassy] → spike passed compile; add runtime test asserting 10 advances = 10 control ticks.
- [Feature unification 1 MHz] → isolate harness features; verify MCU thumb builds.
- [Raw executor order] → only one control task in harness; no reliance on wake order.
- [Physics fidelity scope] → simplified point-mass + foot contacts, not full rigid body; enough for believable walking and controller validation.
- [Bounded catch-up] → if frames cannot sustain 200 Hz, sim runs slower than wall clock but stays deterministic and reports achieved rate.
