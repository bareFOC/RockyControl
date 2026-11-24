# Proposal

## Why

The MCP drawer in the native desktop app advertises `/mcp` as live, but `pump_mcp_bridge()` is a no-op on native — there is no listener, so external MCP clients (Claude Desktop, curl) cannot drive the actual running GUI with its 200 Hz loop, fall-recovery watcher, and live 3D state. This change adds a loopback-only Streamable HTTP MCP listener to `rk-desktop` and bridges requests onto the egui thread via a bounded channel.

## What Changes

- **BREAKING (internal):** Refactor `rk-mcp::http_server` to accept a pluggable dispatcher (`Headless` vs `Bridge`) while preserving `serve_http(addr, HeadlessHost)` for the CLI.
- Add `rk-mcp::bridge` with `McpBridgeTx/Rx`, bounded FIFO requests + oneshot replies, and `mcp_bridge_channel(wake)`; add `spawn_http_bridge(addr, tx) -> HttpServerHandle` with graceful stop.
- Add `mcp_bridge_rx` to `RockyStudioApp` with `attach_mcp_bridge()`; drain in native `pump_mcp_bridge()` (called from `DesktopWrapper::logic` so hidden windows still service requests); gate WASM drain to WASM only.
- Add listener status to `McpUiState` and fix the drawer's false "live" label + broken shove preset args (`strength`/`direction` → `pitch_impulse_rad_s`/`roll_impulse_rad_s`/`step_ticks`).
- Start listener in `rk-desktop main()` on `127.0.0.1:<port>` (default 7788, `--mcp-port`/`ROCKY_MCP_PORT`, `--no-mcp`), non-fatal on port conflict; stop in `on_exit`/Drop.

## Capabilities

### New Capabilities
- `native-mcp-bridge`: External MCP clients can drive the running native desktop app over loopback HTTP, with requests serialized onto the egui thread.

### Modified Capabilities
- `firmware-integration-health`: none.
- No behavior spec changes for control — MCP tools already have specs.

## Impact
- `rk-mcp` (bridge + HTTP refactor), `rk-ui` (app field/drain/drawer), `rk-desktop` (startup/shutdown).
- Security: loopback only, no auth (consistent with existing CLI); CORS `*` unchanged (documented).
- Risk: non-Send app requires channel bridge; long `step_ticks` calls block the UI thread (pre-existing in-process behavior, now exposed over HTTP with a timeout).
