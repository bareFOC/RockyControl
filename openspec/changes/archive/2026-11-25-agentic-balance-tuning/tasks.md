# Tasks

> Sequential phases. Verify green after each. Final: archive + sync + commit + push.

## Phase 1 — feedback loop (OWNER: quality agent; FIRST)
- [x] 1.1 `WalkQuality: Serialize`; `RobotTransport::walk_quality` + SimTransport impl; `McpHost::walk_quality`; `quality_snapshot` tool + resource. Verify `cargo test -p rk-sim -p rk-link -p rk-mcp`.
- [x] 1.2 `run_experiment` uses tuned gains (inject SetBalanceGains on packet 0) and returns `{verification, quality}`; update all callers/tests. Verify golden fixtures still exact.

## Phase 2 — acceptance & recovery (OWNER: acceptance agent; after Phase 1)
- [x] 2.1 `WalkQualityBounds`/`evaluate` in rk-sim; refactor rk-mcp/rk-sim assertions; `quality_snapshot` returns bounds/violations/pass.
- [x] 2.2 `balance_reset_defaults`, `experiment_list`, `experiment_run_named`; structured JSON gain errors. Verify `cargo test --workspace` and live MCP smoke.

## Finish
- [x] 3.1 Full gate: workspace, firmware-core, thumb, WASM, sim-verify, strict validate; archive + sync; commit and push.
