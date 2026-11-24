# Tasks

> Subagent-delegable. No back-compat needed.

## 1. Headless fixed-step (OWNER: rk-mcp agent)
- [x] 1.1 Update `HeadlessHost::step_ticks` in `crates/rk-mcp/src/headless.rs` to call `transport.step_fixed(1, &pkt, FIXED_CONTROL_STEP_SEC)` per tick (variant A). Update `McpHost::step_ticks` rustdoc in `host.rs`. Verify `cargo test -p rk-mcp`.
- [x] 1.2 Add headless test: record 20 ticks via `step_ticks`, assert `dt_sec == 0.005`, `now_ms` increments by 5, returned telemetry still has WORLD/GRF validity bits. Verify `cargo test -p rk-mcp`.

## 2. MCP replay test (OWNER: rk-mcp agent; same files)
- [x] 2.1 Add `mcp_replay_is_bit_exact_after_fixed_steps` test in tools.rs: connect sim → start recording → ≥80 steps → stop → verify_replay → assert exact_match true and matched==total. Keep existing 25-tool, shove, guarded-autonomy tests passing. Verify `cargo test -p rk-mcp`.

## 3. GUI bridge parity (OWNER: ui agent; parallel, disjoint file)
- [x] 3.1 Update `McpHost for RockyStudioApp::step_ticks` in `crates/rk-ui/src/mcp_bridge.rs` to the same fixed-step loop. Verify `cargo check -p rk-ui` and `cargo test -p rk-ui`.

## 4. Verification (OWNER: integrator)
- [x] 4.1 Full host suite (167 tests) + workspace check + lint + strict validate.
- [x] 4.2 Live MCP replay smoke test: 120/120 and 1000/1000 bit-exact over 25 sequential calls, no panic.
