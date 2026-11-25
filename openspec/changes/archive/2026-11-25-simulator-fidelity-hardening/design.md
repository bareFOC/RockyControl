# Design

## Context

The simulator shares the control kernel and deterministic executor with firmware, but its plant, wire and observability layers have concrete defects: GRF channels swapped, velocity commands ignored, `post_tick` never called on sim IO, fixed-step host commands bypass COBS, no reset operation, partial MCP fault coverage, and telemetry samples lacking the signals needed for tuning. Firmware-side behavior is the reference: `RobotIo::post_tick` runs the LED state machine (`rk_fw_core::status_led::StatusLedMode::is_lit`), `decode_rx`/`CobsStreamFramer` handle wire errors, and `SimFaultInjection` in `rk-sim` is the full fault set.

## Goals / Non-Goals

**Goals:** correct signal mapping; firmware semantics on the sim path; reproducible experiments; full fault coverage; live observability of tuning signals.

**Non-Goals:** changing firmware code; changing the control kernel; adding a general physics engine; seeded sensor noise / torque-based dynamics / slip (medium tier, follow-up change); MCU replay export; time-origin normalization (large tier).

## Decisions

1. **GRF mapping** (`rk-sim` telemetry): use `foot_contact(true)` for `grf_left_n` and `foot_contact(false)` for `grf_right_n`, matching `foot_contact(left)` index 1/0. Add an asymmetric-contact regression test.
2. **Velocity mode**: in `step_plant`, when `command_mode[i] == JointCommandMode::Velocity`, drive the joint from `raw_velocity_rad_s[i]` (integrate angle, feed measured velocity back) instead of holding the position target. Keep torque mode and position mode unchanged. Add plant + replay tests.
3. **post_tick**: implement `SimRobotIo::post_tick(now_ms)` evaluating `current_led_mode().is_lit(now_ms)` from `rk_fw_core::copper`, storing a bool in `SimRobotIo`. Expose it through `RobotTransport` as a telemetry/state field and render a small LED indicator in the control HUD. Assert in a test that it fires every tick.
4. **Fixed-step wire path**: in `rk-link::step_fixed_inner`, replace typed-command queueing with `encode_cobs` → `decode_rx` (reuse `firmware/rk-fw-core/src/transport.rs` framer and `rk-sim`'s decode helpers). Add fault knobs for CRC mismatch, version mismatch, truncation and fragmentation; surface counts via `wire_error_counts` in telemetry. Keep the zero-fault path bit-exact.
5. **Reset experiment**: add `CopperSimRunner::reset_experiment()` that resets the mock clock (`reset_mock_clock`), rebuilds kernel, plant, faults and recorder, and clears UI history. Call it from the UI sim reconnect path and expose via MCP.
6. **Fault parity**: extend `sim_inject_fault` MCP tool to cover all `SimFaultInjection` fields plus the new wire faults; keep one shared enum so UI and MCP cannot drift.
7. **Observability**: extend `TelemetryPoint` with `com_x_m`, `com_z_m`, `grf_left_n`, `grf_right_n`, `contact_mask`, `tilt_rate_dps`; add plots in `config_screen.rs` next to pitch/roll; color feet by contact/GRF in `rk-viewport`. Note in docs that WORLD/GRF channels are sim-only enrichment (hardware v1 leaves them zero).

## Risks / Trade-offs

- Adding COBS to the fixed-step path changes command handoff timing; must preserve bit-exact replay (test explicitly).
- `post_tick` LED state adds a sim-visible side effect; keep it out of recorded kernel ticks.
- Reset operation must not reset the MCP bridge channel or break the persistent executor contract.
- New telemetry fields are additive; WASM/native compatibility must be retained.
