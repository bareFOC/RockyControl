# Design

## Context

`serve_http` is hard-wired to `HeadlessHost` (Send) and owns it in an `Arc<Mutex<..>>`; `RockyStudioApp` is not Send and lives inside eframe's creation closure. `dispatch_json_rpc<H: McpHost>` has no Send bound and must run on the owning thread. `egui::Context::request_repaint()` is thread-safe, and `eframe::App::logic` runs even when the window is hidden — so the native drain belongs in `logic`, not `update_frame`.

## Goals / Non-Goals

**Goals:** external clients drive the live GUI; loopback only; graceful shutdown; honest UI status; preserve CLI `rk-mcp --http` behavior. **Non-Goals:** making the app Send; true SSE notifications (existing GET /mcp is one-shot); auth/TLS.

## Decisions

1. **Bounded channel + oneshot replies** (`rk-mcp::bridge`): `McpBridgeRequest { message, reply: Sender<Option<Value>> }`; `McpBridgeTx` with `SyncSender` (capacity 16) + thread-safe wake callback; `submit()` uses `try_send` + `recv_timeout(15s)`. `None` reply → HTTP 202 (notification); Busy → 503; timeout/GuiClosed → 504/503.
2. **Dispatcher enum in http_server**: `Headless(Arc<Mutex<HeadlessHost>>)` (CLI) vs `Bridge(McpBridgeTx)` (GUI). Keep `serve_http(addr, HeadlessHost)` as blocking wrapper; add `spawn_http_bridge(addr, tx) -> HttpServerHandle` with nonblocking listener + stop flag + JoinHandle, binding synchronously to report conflicts.
3. **Drain in `DesktopWrapper::logic`** before `craft_quit::on_close_request`; native `pump_mcp_bridge()` drains the queue and calls `dispatch_json_rpc(self, req.message)`; WASM branch untouched, `update_frame:713` call gated to WASM.
4. **App field** `#[cfg(not(wasm32))] mcp_bridge_rx: Option<McpBridgeRx>` + `attach_mcp_bridge()`; borrow pattern: collect requests into a local Vec first, then dispatch.
5. **Startup** in `main()`: parse `--mcp-port`/`ROCKY_MCP_PORT`/`--no-mcp`, spawn listener before run_native, store handle in wrapper, `on_exit` stops it, Drop backup. Non-fatal port conflict with status in drawer.
6. **Drawer fixes**: status-dependent endpoint label; shove args `pitch_impulse_rad_s: push_strength.impulse_rad_s()`, `roll_impulse_rad_s: -0.9`, `step_ticks: 260`.

## Risks / Trade-offs

- `step_ticks(400)` runs synchronously on the egui thread → can stall rendering; bounded by 15 s HTTP timeout (pre-existing in-process behavior now exposed).
- `Access-Control-Allow-Origin: *` unchanged — any local webpage can call loopback /mcp; documented, not hardened.
- craft-quit can terminate before `on_exit`; OS closes socket; waiting handlers see dropped receiver.
- ~250–350 LOC across 6 files; moderate complexity.
