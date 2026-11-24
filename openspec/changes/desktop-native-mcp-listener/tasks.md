# Tasks

> Delegable. Group A first, then B and C in parallel.

## A. rk-mcp bridge + HTTP refactor (OWNER: mcp agent; files: rk-mcp/src/{bridge.rs new, http_server.rs, lib.rs})
- [x] A1 Add `bridge.rs` (McpBridgeTx/Rx, bounded FIFO, oneshot replies, Busy/Timeout/GuiClosed errors) with unit tests. Verify `cargo test -p rk-mcp`.
- [x] A2 Refactor `http_server.rs` to Dispatcher enum (Headless preserved; Bridge via McpBridgeTx), add `spawn_http_bridge` + `HttpServerHandle`, split sessions mutex from dispatch wait. Preserve `serve_http` CLI. Verify `cargo test -p rk-mcp` + real HTTP test with ephemeral port.
- [x] A3 Export bridge types from rk-mcp lib.rs (native only).

## B. rk-ui live app integration (OWNER: ui agent; files: rk-ui/src/{mcp_bridge.rs, lib.rs, mcp_drawer.rs})
- [x] B1 Add `mcp_bridge_rx` field + `attach_mcp_bridge()` to RockyStudioApp; native drain in `pump_mcp_bridge` (collect first, then dispatch); gate `update_frame:713` drain to WASM. Verify `cargo test -p rk-ui`.
- [x] B2 Add NativeMcpListenerStatus + drawer status-dependent label; fix shove preset args. Verify `cargo test -p rk-ui`.
- [x] B3 Add unit test: new_app + attach bridge + enqueue tools/list + pump → reply.

## C. rk-desktop startup/shutdown (OWNER: desktop agent; files: apps/rk-desktop/{main.rs, Cargo.toml}; depends on A)
- [x] C1 Add port CLI/env parsing, listener spawn in creation closure, `logic()` drain, `on_exit` stop, non-fatal conflict. Verify `cargo check -p rk-desktop` and `cargo check --workspace`.

## D. Verification (OWNER: integrator)
- [x] D1 Full host suite + firmware-core + thumb + WASM check + lint + strict validate.
- [x] D2 Manual smoke: run desktop with `--mcp-port 7790`, curl /health + /mcp tools over live GUI; verify 3D state changes.
