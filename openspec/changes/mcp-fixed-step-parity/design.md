# Design

## Context

`step_fixed(steps, host_pkt, frame_dt)` exists in `rk-link` and drives the shared Embassy task at fixed 5 ms with bit-exact replay. Headless MCP (`headless.rs:241-248`) and the GUI's in-process MCP host (`mcp_bridge.rs:471-478`) still loop with `step_exchange(0.01)`, producing variable-dt ticks and replay mismatch (0/80).

## Goals / Non-Goals

**Goals:** both MCP hosts use `step_fixed`; replay exact over MCP; tool semantics preserved. **Non-Goals:** batching one packet for N ticks (would alter playback/config semantics); wire/protocol changes; rk-link/rk-sim/firmware changes.

## Decisions

1. **Variant A (behavior-preserving):** keep `build_next_host_packet(dt)` once per tick, call `transport.step_fixed(1, &pkt, FIXED_CONTROL_STEP_SEC)` per tick, then ingest returned telemetry. This keeps per-tick action playback advancement, pending_config drain, tx_seq increments, and `sim_push_shove` peak sampling intact.
2. Keep `McpHost::step_ticks(ticks, dt_sec)` signature; update rustdoc to describe fixed 5 ms ticks. `dt_sec` remains the caller-provided value used for packet building / playback timing.
3. `frame_dt_sec` for rate reporting is `FIXED_CONTROL_STEP_SEC` (0.005) per tick → achieved rate reports 200 Hz.
4. Returned telemetry stays plant-augmented (WORLD/GRF); recorded `ControlTick`s remain kernel-exact (existing design in `SimRobotIo::publish_telemetry`/`record_tick`).
5. No batching: tool `step_ticks` 0..400 may spawn up to 400 harness runs; acceptable for MCP (measure in tests).

## Risks / Trade-offs

- 400 harness spawns per shove tool call — measure; if too slow, follow-up with variant B + shove rewrite.
- Mock clock is global; tests serialize with a lock / explicit reset.
- GUI and headless now share fixed-step semantics; any future divergence must be avoided.
