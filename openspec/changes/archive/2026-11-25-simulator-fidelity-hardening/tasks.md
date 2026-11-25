# Tasks

> Delegable. Group 1 first (foundation); Groups 2–4 in parallel by file owner; Group 5 integrates.

## 1. Foundation — GRF + velocity + post_tick (OWNER: sim agent; FIRST)
- [x] 1.1 Fix GRF left/right swap in `crates/rk-sim/src/lib.rs` telemetry; add asymmetric-contact test.
- [x] 1.2 Honor `JointCommandMode::Velocity` in `step_plant` using `raw_velocity_rad_s`; add plant and replay tests.
- [x] 1.3 Implement `SimRobotIo::post_tick` evaluating the firmware LED state machine; store/expose `sim_led_on`; add a tick-fires test. Verify `cargo test -p rk-sim`.

## 2. Wire path (OWNER: wire agent; depends on 1)
- [x] 2.1 Route `rk-link` fixed-step host commands through COBS encode/`decode_rx`; add CRC/version/truncation/fragmentation fault knobs in `rk-sim`; surface counts via `wire_error_counts`. Verify `cargo test -p rk-link -p rk-sim`.

## 3. MCP + UX (OWNER: mcp/ui agent; depends on 1, parallel to 2)
- [x] 3.1 Add reset-experiment operation (`reset_mock_clock` + kernel/plant/faults/history) and call it on UI reconnect; expose via MCP.
- [x] 3.2 Extend `sim_inject_fault` MCP tool to all `SimFaultInjection` fields + new wire faults; add parity test.
- [x] 3.3 Render LED indicator in control HUD. Verify `cargo test -p rk-mcp -p rk-ui`.

## 4. Observability (OWNER: ui/viewport agent; depends on 1, parallel to 2/3)
- [x] 4.1 Extend `TelemetryPoint` with CoM/GRF/contact/tilt-rate; add plots in `config_screen.rs`.
- [x] 4.2 Color per-foot contact/GRF in `rk-viewport`; add doc note that WORLD/GRF are sim-only. Verify `cargo check --workspace`.

## 5. Verification (OWNER: integrator; depends on all)
- [x] 5.1 Full host suite, firmware-core tests, WASM check, lint, strict validate. Update change status.
