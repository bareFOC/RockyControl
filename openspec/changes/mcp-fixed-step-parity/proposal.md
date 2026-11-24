# Proposal

## Why

MCP-driven simulation currently steps via variable-dt `step_exchange(0.01)`, while the GUI uses the deterministic 200 Hz `step_fixed` path. As a result, `recording_control(verify_replay)` over MCP returns 0/80 matched ticks (recorded ticks carry plant-augmented/variable-time state a fresh kernel cannot reproduce). This change makes headless MCP and the GUI's in-process MCP host use the same fixed-step path so replay is bit-exact and headless behavior matches the app.

## What Changes

- Change `HeadlessHost::step_ticks` and `McpHost for RockyStudioApp::step_ticks` to call `RobotTransport::step_fixed(1, &pkt, FIXED_CONTROL_STEP_SEC)` per tick (behavior-preserving variant A: keep one `build_next_host_packet(dt)` per tick so action playback, config drain, seq increments, and shove peak sampling remain unchanged).
- Update `McpHost::step_ticks` docs to state fixed 5 ms ticks; no signature change (all ~20 tool call sites untouched).
- Add MCP replay test: connect sim → start recording → step ≥80 ticks → stop → verify_replay, asserting `exact_match == true`.
- Add headless test asserting recorded ticks have `dt_sec == 0.005` and `now_ms` increments by 5, while returned telemetry still carries plant WORLD/GRF validity.
- No wire, protocol, `rk-link`, `rk-sim`, or firmware changes: `step_fixed` and bit-exact replay already exist.

## Capabilities

### New Capabilities
- `mcp-deterministic-stepping`: MCP tool calls advance the simulation with the same deterministic fixed-step control path as the GUI, making replay verification bit-exact.

### Modified Capabilities
- `unified-control-application`: the MCP host path uses the shared fixed-step control application instead of the variable-dt compatibility path.

## Impact
- `crates/rk-mcp/src/{headless.rs, host.rs, tools.rs (tests)}`, `crates/rk-ui/src/mcp_bridge.rs`.
- Risk: tool behavior changes (dt semantics, rate reporting); mitigated by variant A preserving per-tick packet building.
